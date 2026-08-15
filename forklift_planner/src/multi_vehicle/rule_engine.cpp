#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/conflict_zone_closure.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/future_a1_policy.h"

namespace forklift_planner {
namespace multi_vehicle {

RuleEngine::RuleEngine(const MapParam& mp, const MultiVehicleConfig& cfg)
    : mp_(mp), cfg_(cfg) {}

bool RuleEngine::shouldLogA1Decision(const VehicleAgent& vehicle,
                                     int blocker_id) {
    return a1_decision_logs_.insert(std::make_tuple(
        vehicle.id, blocker_id, static_cast<int>(vehicle.mission_phase),
        static_cast<int>(vehicle.requested_action))).second;
}

std::string RuleEngine::debugLogPrefix() const {
    std::ostringstream out;
    out << "[SOURCE=" << debug_log_source_ << "]"
        << " [plan=" << debug_log_plan_id_ << "]"
        << " [frame=" << debug_log_frame_id_ << "]"
        << " [rollout_step=" << debug_log_rollout_step_ << "]";
    return out.str();
}

namespace {

std::vector<ConflictMarker::TimedOverlap> decimateTimedOverlaps(
    const std::vector<ConflictMarker::TimedOverlap>& input) {
    constexpr size_t kMaxDisplayedSamples = 32;
    if (input.size() <= kMaxDisplayedSamples) return input;
    std::vector<ConflictMarker::TimedOverlap> output;
    output.reserve(kMaxDisplayedSamples);
    for (size_t i = 0; i < kMaxDisplayedSamples; ++i) {
        const size_t index = i * (input.size() - 1) /
                             (kMaxDisplayedSamples - 1);
        output.push_back(input[index]);
    }
    return output;
}

struct OverlapSample {
    double s_self = 0.0;
    double s_other = 0.0;
    double x = 0.0;
    double y = 0.0;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    bool bounds_valid = false;
};

const char* missionPhaseName(MissionPhase phase) {
    switch (phase) {
        case MissionPhase::DIRECT_TO_B: return "DIRECT_TO_B";
        case MissionPhase::TO_A1: return "TO_A1";
        case MissionPhase::PICKUP_DWELL: return "PICKUP_DWELL";
        case MissionPhase::WAIT_DROPOFF_TASK: return "WAIT_DROPOFF_TASK";
        case MissionPhase::TO_B: return "TO_B";
        case MissionPhase::UNLOAD_DWELL: return "UNLOAD_DWELL";
    }
    return "UNKNOWN";
}

void logConflictReservation(
    const std::function<void(const std::string&)>& sink,
    const std::pair<int, int>& key, const char* op,
    const RuleEngine::ConflictReservation& reservation) {
    if (!sink) return;
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[CONFLICT_RESERVATION] component=PAIRWISE pair=V" << key.first
         << "-V" << key.second << " event=" << op
         << " holder=V" << reservation.owner_id
         << " waiter=V"
         << (reservation.owner_id == key.first ? key.second : key.first)
         << " conflict_type=CROSSING/OPPOSING"
         << " t=" << reservation.first_conflict_t
         << " zone=(" << reservation.x << "," << reservation.y << ")";
    sink(line.str());
}

bool sameReservation(const RuleEngine::ConflictReservation& a,
                     const RuleEngine::ConflictReservation& b) {
    return a.owner_id == b.owner_id &&
           a.gen_lo == b.gen_lo && a.gen_hi == b.gen_hi &&
           a.enter_lo == b.enter_lo && a.exit_lo == b.exit_lo &&
           a.enter_hi == b.enter_hi && a.exit_hi == b.exit_hi &&
           a.x == b.x && a.y == b.y &&
           a.first_conflict_t == b.first_conflict_t &&
           a.raw_zone_index == b.raw_zone_index &&
           a.aabb_min_x == b.aabb_min_x && a.aabb_min_y == b.aabb_min_y &&
           a.aabb_max_x == b.aabb_max_x && a.aabb_max_y == b.aabb_max_y &&
           a.aabb_valid == b.aabb_valid;
}

bool sameDepartureCluster(
    const RuleEngine::DepartureClusterCommitment& a,
    const RuleEngine::DepartureClusterCommitment& b) {
    return a.owner_id == b.owner_id &&
           a.owner_path_gen == b.owner_path_gen &&
           a.other_id == b.other_id &&
           a.other_path_gen == b.other_path_gen &&
           a.seed_indices == b.seed_indices &&
           a.cluster_indices == b.cluster_indices &&
           a.waiter_stop_boundary_s == b.waiter_stop_boundary_s &&
           a.waiter_stop_s == b.waiter_stop_s &&
           a.owner_release_exit_s == b.owner_release_exit_s &&
           a.other_release_exit_s == b.other_release_exit_s &&
           a.active == b.active &&
           a.handed_off_from_future == b.handed_off_from_future &&
           a.handoff_already_inside == b.handoff_already_inside;
}

void logDepartureCluster(
    const std::function<void(const std::string&)>& sink,
    const char* event, const char* reason,
    const RuleEngine::DepartureClusterCommitment& commitment,
    double owner_s, double other_s) {
    if (!sink) return;
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[DEPARTURE_CLUSTER] event=" << event
         << " reason=" << reason
         << " owner=V" << commitment.owner_id
         << " owner_gen=" << commitment.owner_path_gen
         << " other=V" << commitment.other_id
         << " other_gen=" << commitment.other_path_gen
         << " zones=[";
    for (size_t i = 0; i < commitment.cluster_indices.size(); ++i) {
        if (i > 0) line << ",";
        line << commitment.cluster_indices[i];
    }
    line << "] waiter_stop_boundary_s="
         << commitment.waiter_stop_boundary_s
         << " stop_s=" << commitment.waiter_stop_s
         << " owner_release_exit_s=" << commitment.owner_release_exit_s
         << " owner_s=" << owner_s
         << " other_s=" << other_s
         << " future_handoff="
         << (commitment.handed_off_from_future ? "true" : "false")
         << " already_inside="
         << (commitment.handoff_already_inside ? "true" : "false");
    sink(line.str());
}

void logA1Decision(const std::function<void(const std::string&)>& sink,
                   const MultiVehicleConfig& cfg, const VehicleAgent& vehicle,
                   const VehicleAgent* blocker, int blocker_id) {
    if (!sink || vehicle.track.empty()) return;
    const RoughWp pose = vehicle.track.poseAtS(vehicle.path_s);
    const double dist_to_a1 = std::hypot(
        pose.x - cfg.a1_pickup_center_x,
        pose.y - cfg.a1_pickup_center_y);
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[A1_DECISION] V" << vehicle.id
         << " action=" << actionName(vehicle.requested_action)
         << " reason=" << vehicle.reason
         << " blocker=V" << blocker_id
         << " phase=" << missionPhaseName(vehicle.mission_phase)
         << " dist_to_A1=" << dist_to_a1
         << " blocker_phase="
         << (blocker ? missionPhaseName(blocker->mission_phase) : "UNKNOWN");
    sink(line.str());
}

}  // namespace

RuleEngine::SimSnapshot RuleEngine::snapshot() const {
    return SimSnapshot{conflict_reservations_, departure_cluster_commitments_,
                       following_pairs_, tokens_, conflicts_, now_};
}

void RuleEngine::restore(const SimSnapshot& s) {
    for (const auto& current : conflict_reservations_) {
        if (s.reservations.count(current.first) == 0) {
            logConflictReservation(coord_log_sink_, current.first, "delete",
                                   current.second);
        }
    }
    for (const auto& incoming : s.reservations) {
        const auto current = conflict_reservations_.find(incoming.first);
        if (current == conflict_reservations_.end()) {
            logConflictReservation(coord_log_sink_, incoming.first, "create",
                                   incoming.second);
        } else if (!sameReservation(current->second, incoming.second)) {
            logConflictReservation(coord_log_sink_, incoming.first, "update",
                                   incoming.second);
        }
    }
    for (const auto& current : departure_cluster_commitments_) {
        const auto incoming = s.departure_clusters.find(current.first);
        if (current.second.active &&
            (incoming == s.departure_clusters.end() ||
             !incoming->second.active)) {
            logDepartureCluster(coord_log_sink_, "RELEASE", "snapshot_restore",
                                current.second, -1.0, -1.0);
        }
    }
    for (const auto& incoming : s.departure_clusters) {
        const auto current = departure_cluster_commitments_.find(incoming.first);
        if (incoming.second.active &&
            (current == departure_cluster_commitments_.end() ||
             !current->second.active)) {
            logDepartureCluster(coord_log_sink_, "CREATE", "snapshot_restore",
                                incoming.second, -1.0, -1.0);
        } else if (incoming.second.active && current != departure_cluster_commitments_.end() &&
                   !sameDepartureCluster(current->second, incoming.second)) {
            logDepartureCluster(coord_log_sink_, "HOLD", "snapshot_restore",
                                incoming.second, -1.0, -1.0);
        }
    }
    conflict_reservations_ = s.reservations;
    departure_cluster_commitments_ = s.departure_clusters;
    following_pairs_ = s.following_pairs;
    tokens_ = s.tokens;
    conflicts_ = s.conflicts;
    now_ = s.now;
}

double RuleEngine::speedForAction(VehicleAction action) const {
    switch (action) {
        case VehicleAction::STOP:
            return 0.0;
        case VehicleAction::CREEP:
            return cfg_.nominal_speed * cfg_.creep_ratio;
        case VehicleAction::YIELD:
            return cfg_.nominal_speed * cfg_.yield_ratio;
        case VehicleAction::NOMINAL:
            return cfg_.nominal_speed;
        case VehicleAction::BOOST:
            return cfg_.enable_boost
                       ? std::min(cfg_.max_speed,
                                  cfg_.nominal_speed * cfg_.boost_ratio)
                       : cfg_.nominal_speed;
    }
    return 0.0;
}

int RuleEngine::priorityWinner(const VehicleAgent& a,
                               const VehicleAgent& b) const {
    if (!cfg_.enable_priority_tiebreak) return -1;

    // §9 破环车临时最高优先级:跨层一致——pairwise 也必须让破环车赢,否则它在资源层
    // 赢了令牌、却被 pairwise 摁住,环还是破不了。
    if (a.deadlock_breaker != b.deadlock_breaker) return a.deadlock_breaker ? a.id : b.id;

    // 资源前置约束(非任意优先级,§4/§6/§7):若一车要去的目标库位正被另一车占着
    // (a.target==b.current),占用者必须先清出该位、入库者让行——否则入库者抢先开到
    // 库位口却进不去(位被占),占用者又被它让停在口内,直接死锁。这是 slot 资源依赖,
    // 当任务只指向空/即将空的库位时无环;故置于严格全序之上作 override(若与全序冲突成环
    // =任务分配层的循环占位问题,由第5步环检测断言抓出)。
    {
        const bool a_wants_b_slot = (a.target_slot == b.current_slot);
        const bool b_wants_a_slot = (b.target_slot == a.current_slot);
        if (a_wants_b_slot && !b_wants_a_slot) return b.id;  // b 占用者,先清出
        if (b_wants_a_slot && !a_wants_b_slot) return a.id;  // a 占用者,先清出
    }

    auto motionHeading = [](const VehicleAgent& v) {
        constexpr double kPi = 3.14159265358979323846;
        double heading = v.track.poseAtS(v.path_s).theta;
        if (v.track.typeAtS(v.path_s) == WpType::REVERSE) heading += kPi;
        return heading;
    };

    const double ha = motionHeading(a);
    const double hb = motionHeading(b);
    const double dot =
        std::cos(ha) * std::cos(hb) + std::sin(ha) * std::sin(hb);
    if (dot <= 0.70) return unifiedPriority(a, b);

    const RoughWp pa = a.track.poseAtS(a.path_s);
    const RoughWp pb = b.track.poseAtS(b.path_s);
    const double dx = pb.x - pa.x;
    const double dy = pb.y - pa.y;
    const double lat = std::abs(-dx * std::sin(ha) + dy * std::cos(ha));
    if (lat > mp_.vehicle_width) return unifiedPriority(a, b);

    const double fwd = dx * std::cos(ha) + dy * std::sin(ha);
    if (std::abs(fwd) < 1e-6) return unifiedPriority(a, b);
    return fwd > 0.0 ? b.id : a.id;
}

int RuleEngine::unifiedPriority(const VehicleAgent& a,
                                const VehicleAgent& b) const {
    // 协调图第2步:严格全序 ≺。按可证「反对称+传递+完全」的字典序键比较,id 作唯一终裁。
    // 任意时刻"谁让谁"关系由全序导出 ⇒ 必无环 ⇒ 无死锁(见 草履虫规则_协调图统一架构
    // 设计 §2.3/§5)。
    // 键(越小越优先 / 越先走):
    //   1) 饥饿组(§17):wait_time 超阈值者整体提升一档(有界提升);组内等得越久越先,
    //      以防长饥饿。非饥饿组此分量恒 0、不参与比较。
    //   2) 载货优先;3) 已完成任务少者优先(摊平工作量);4) id 最小(确定性终裁)。
    // 注:原"占用者先清出"的成对 slot 规则会破坏传递性(成对、非全序),已移出本函数,
    //     在 priorityWinner 中作显式**资源前置约束** override(见那里)。原 both-starved 的
    //     1e-3 eps 相等判定也是非传递的,这里改为直接比较、由 id 终裁,彻底消除非传递性。
    auto key = [&](const VehicleAgent& v) {
        const bool starved = v.wait_time > cfg_.starvation_wait_time;
        return std::make_tuple(starved ? 0 : 1,             // 饥饿组优先
                               starved ? -v.wait_time : 0.0, // 组内久者优先
                               v.loaded ? 0 : 1,             // 载货优先
                               v.task_count,                 // 任务少优先
                               v.id);                        // 唯一终裁
    };
    return key(a) < key(b) ? a.id : b.id;
}

double RuleEngine::predictedTravelDistance(const VehicleAgent& v,
                                           VehicleAction action,
                                           double t) const {
    if (t <= 0.0) return 0.0;

    const double desired = speedForAction(action);
    const double current = std::max(0.0, v.current_speed);
    if (std::abs(desired - current) < 1e-9) {
        return current * t;
    }

    const bool accelerating = desired > current;
    const double limit = accelerating ? cfg_.max_accel : cfg_.max_decel;
    if (limit <= 1e-9) {
        return current * t;
    }

    const double signed_accel = accelerating ? limit : -limit;
    const double ramp_time = std::abs(desired - current) / limit;
    const double ramp_used = std::min(t, ramp_time);
    const double ramp_dist =
        current * ramp_used + 0.5 * signed_accel * ramp_used * ramp_used;
    if (t <= ramp_time) {
        return std::max(0.0, ramp_dist);
    }
    return std::max(0.0, ramp_dist + desired * (t - ramp_time));
}


std::vector<RuleEngine::ConflictZone> RuleEngine::computeConflictZonesFull(
    const VehicleAgent& self, const VehicleAgent& other) const {
    constexpr double kStep = 0.025;
    constexpr double kMergeGap = kStep * 2.25;
    std::vector<ConflictZone> zones;

    // 两条路径都是「完全固定、已知」的;它们之间的冲突集 C_ij 是静态几何量,与时间/
    // 速度/朝向/当前位置都无关 → 在整段路径 [0,L]×[0,L] 上一次算定即可(由
    // conflictBlocksCanonical 按 path_gen 缓存)。当前位置相关的裁剪(裁掉已清出的块、
    // 把入口夹到车尾起点)在 findConflictZones 里按调用时的 path_s 施加。
    // (历史教训:早先按 current_speed×prediction_horizon 裁剪扫描范围,会在让行车停在
    // 停止线 speed→0 时让冲突区凭空消失 → 原子门预约被释放 → 让行车蹭过停止线挤进区
    // → 三车旋转楔死。改扫整段固定路径根治;此处进一步缓存,几何恒定可见且不每拍重算。)
    const double s_self_end = self.track.length();
    const double s_other_end = other.track.length();
    const double s_self_begin = 0.0;
    const double s_other_begin = 0.0;

    // 广相剪枝(扫全程后的性能护栏):两车「剩余路径」的轴对齐包围盒各按「车身对角线
    // 半径 + 冲突余量」外胀;若两盒分离,则任意位姿下车身 OBB 绝无可能重叠 → 直接返回空,
    // 免去对八竿子打不着的远车做整段精扫。这只省算力,绝不改变任何冲突判定结果。
    {
        const double inf = std::numeric_limits<double>::infinity();
        const double infl =
            0.5 * std::hypot(mp_.vehicle_length, mp_.vehicle_width) +
            cfg_.conflict_margin;
        constexpr double kCoarse = 0.15;  // 粗采样建包围盒
        double ax0 = inf, ay0 = inf, ax1 = -inf, ay1 = -inf;
        for (double s = s_self_begin; s <= s_self_end + 1e-9; s += kCoarse) {
            const RoughWp p = self.track.poseAtS(std::min(s, s_self_end));
            ax0 = std::min(ax0, p.x); ay0 = std::min(ay0, p.y);
            ax1 = std::max(ax1, p.x); ay1 = std::max(ay1, p.y);
        }
        double bx0 = inf, by0 = inf, bx1 = -inf, by1 = -inf;
        for (double s = s_other_begin; s <= s_other_end + 1e-9; s += kCoarse) {
            const RoughWp p = other.track.poseAtS(std::min(s, s_other_end));
            bx0 = std::min(bx0, p.x); by0 = std::min(by0, p.y);
            bx1 = std::max(bx1, p.x); by1 = std::max(by1, p.y);
        }
        if (ax1 + infl < bx0 - infl || bx1 + infl < ax0 - infl ||
            ay1 + infl < by0 - infl || by1 + infl < ay0 - infl) {
            return zones;  // 包围盒分离 → 无冲突
        }
    }

    for (double ss = s_self_begin; ss <= s_self_end + 1e-9; ss += kStep) {
        const double ss_clamped = std::min(ss, s_self_end);
        // 冲突检测加安全余量：每车身各胀 conflict_margin/2，总净距 < conflict_margin
        // 即判为冲突 → 提前刹住、留出安全距离(真实开车的"别贴太近")。余量须 < 双车道
        // 会车净间隙(本图 0.059m)，否则会把正常对向会车也误判成冲突。硬护栏仍用 0 余量
        // 作真碰撞底线。
        const double cm = cfg_.conflict_margin * 0.5;
        const OBB obb_s = makeBody(self.track.poseAtS(ss_clamped), mp_, cm);
        std::vector<OverlapSample> row;

        for (double so = s_other_begin; so <= s_other_end + 1e-9; so += kStep) {
            const double so_clamped = std::min(so, s_other_end);
            const OBB obb_o = makeBody(other.track.poseAtS(so_clamped), mp_, cm);

            if (!overlaps(obb_s, obb_o)) continue;

            const RoughWp ps = self.track.poseAtS(ss_clamped);
            const RoughWp po = other.track.poseAtS(so_clamped);
            OverlapSample sample;
            sample.s_self = ss_clamped;
            sample.s_other = so_clamped;
            sample.x = 0.5 * (ps.x + po.x);
            sample.y = 0.5 * (ps.y + po.y);
            const auto polygon = intersectObbs(obb_s, obb_o);
            if (polygon.size() >= 3) {
                sample.min_x = sample.max_x = polygon.front().x;
                sample.min_y = sample.max_y = polygon.front().y;
                for (const auto& point : polygon) {
                    sample.min_x = std::min(sample.min_x, point.x);
                    sample.min_y = std::min(sample.min_y, point.y);
                    sample.max_x = std::max(sample.max_x, point.x);
                    sample.max_y = std::max(sample.max_y, point.y);
                }
                sample.bounds_valid = true;
            }
            row.push_back(sample);
        }

        if (row.empty()) continue;

        std::vector<ConflictZone> row_zones;
        for (const OverlapSample& sample : row) {
            if (row_zones.empty() ||
                sample.s_other > row_zones.back().s_other_exit + kMergeGap) {
                ConflictZone z;
                z.s_self_enter = sample.s_self;
                z.s_self_exit = sample.s_self;
                z.s_other_enter = sample.s_other;
                z.s_other_exit = sample.s_other;
                z.x = sample.x;
                z.y = sample.y;
                z.aabb_min_x = sample.min_x;
                z.aabb_min_y = sample.min_y;
                z.aabb_max_x = sample.max_x;
                z.aabb_max_y = sample.max_y;
                z.aabb_valid = sample.bounds_valid;
                row_zones.push_back(z);
            } else {
                ConflictZone& z = row_zones.back();
                z.s_other_exit = sample.s_other;
                z.x = 0.5 * (z.x + sample.x);
                z.y = 0.5 * (z.y + sample.y);
                if (sample.bounds_valid) {
                    if (!z.aabb_valid) {
                        z.aabb_min_x = sample.min_x;
                        z.aabb_min_y = sample.min_y;
                        z.aabb_max_x = sample.max_x;
                        z.aabb_max_y = sample.max_y;
                    } else {
                        z.aabb_min_x = std::min(z.aabb_min_x, sample.min_x);
                        z.aabb_min_y = std::min(z.aabb_min_y, sample.min_y);
                        z.aabb_max_x = std::max(z.aabb_max_x, sample.max_x);
                        z.aabb_max_y = std::max(z.aabb_max_y, sample.max_y);
                    }
                    z.aabb_valid = true;
                }
            }
        }

        const auto touches = [kMergeGap](const ConflictZone& lhs,
                                         const ConflictZone& rhs) {
            const bool self_touch =
                lhs.s_self_enter <= rhs.s_self_exit + kMergeGap &&
                lhs.s_self_exit + kMergeGap >= rhs.s_self_enter;
            const bool other_touch =
                lhs.s_other_enter <= rhs.s_other_exit + kMergeGap &&
                lhs.s_other_exit + kMergeGap >= rhs.s_other_enter;
            return self_touch && other_touch;
        };
        const auto merge = [](ConflictZone& destination,
                              const ConflictZone& source) {
            destination.s_self_enter =
                std::min(destination.s_self_enter, source.s_self_enter);
            destination.s_self_exit =
                std::max(destination.s_self_exit, source.s_self_exit);
            destination.s_other_enter =
                std::min(destination.s_other_enter, source.s_other_enter);
            destination.s_other_exit =
                std::max(destination.s_other_exit, source.s_other_exit);
            destination.x = 0.5 * (destination.x + source.x);
            destination.y = 0.5 * (destination.y + source.y);
            if (source.aabb_valid) {
                if (!destination.aabb_valid) {
                    destination.aabb_min_x = source.aabb_min_x;
                    destination.aabb_min_y = source.aabb_min_y;
                    destination.aabb_max_x = source.aabb_max_x;
                    destination.aabb_max_y = source.aabb_max_y;
                } else {
                    destination.aabb_min_x =
                        std::min(destination.aabb_min_x, source.aabb_min_x);
                    destination.aabb_min_y =
                        std::min(destination.aabb_min_y, source.aabb_min_y);
                    destination.aabb_max_x =
                        std::max(destination.aabb_max_x, source.aabb_max_x);
                    destination.aabb_max_y =
                        std::max(destination.aabb_max_y, source.aabb_max_y);
                }
                destination.aabb_valid = true;
            }
        };

        for (const ConflictZone& row_zone : row_zones) {
            insertConflictZoneWithClosure(row_zone, zones, touches, merge);
        }
    }

    // 为每个块算「同向」标志(静态):在块中点测两路径行进朝向(REVERSE 段切线+π),
    // 同向(dot>0.7)=正对角带=同车道跟车;否则交叉/对向。供 resolveFollowing 与 pairwise
    // 共用作稳定判据——不再用随当前位姿闪烁的瞬时朝向(交叉/汇入处会瞬时对齐而误判)。
    constexpr double kPi = 3.14159265358979323846;
    auto pathHeadingAtS = [&](const VehicleAgent& v, double s) {
        double h = v.track.poseAtS(s).theta;
        if (v.track.typeAtS(s) == WpType::REVERSE) h += kPi;
        return h;
    };
    for (size_t raw_index = 0; raw_index < zones.size(); ++raw_index) {
        ConflictZone& z = zones[raw_index];
        z.raw_index = static_cast<int>(raw_index);
        const double hs =
            pathHeadingAtS(self, 0.5 * (z.s_self_enter + z.s_self_exit));
        const double ho =
            pathHeadingAtS(other, 0.5 * (z.s_other_enter + z.s_other_exit));
        z.same_dir =
            (std::cos(hs) * std::cos(ho) + std::sin(hs) * std::sin(ho)) > 0.7;
    }

    return zones;
}

RuleEngine::FutureA1ZoneSelection RuleEngine::selectFutureA1ProtectedZones(
    const std::vector<ConflictZone>& canonical_zones,
    bool preview_is_lo, double protected_until,
    double other_path_s) const {
    FutureA1ZoneSelection selection;
    selection.normalized_zones.reserve(canonical_zones.size());
    std::vector<FutureA1ConflictInterval> intervals;
    intervals.reserve(canonical_zones.size());
    for (const ConflictZone& canonical : canonical_zones) {
        ConflictZone normalized = canonical;
        if (!preview_is_lo) {
            std::swap(normalized.s_self_enter, normalized.s_other_enter);
            std::swap(normalized.s_self_exit, normalized.s_other_exit);
        }
        selection.normalized_zones.push_back(normalized);
        intervals.push_back(FutureA1ConflictInterval{
            normalized.s_self_enter, normalized.s_self_exit,
            normalized.s_other_enter, normalized.s_other_exit});
    }

    const FutureA1ProtectedCluster cluster =
        selectFutureA1ProtectedCluster(intervals, protected_until,
                                       other_path_s);
    selection.seed_indices = cluster.seed_indices;
    selection.protected_indices = cluster.protected_indices;
    selection.other_already_inside = cluster.other_already_inside;
    if (cluster.upstream_other_enter) {
        for (size_t index : selection.protected_indices) {
            if (std::abs(selection.normalized_zones[index].s_other_enter -
                         *cluster.upstream_other_enter) <= 1e-9) {
                selection.upstream_index = static_cast<int>(index);
                break;
            }
        }
    }
    return selection;
}

const std::vector<RuleEngine::ConflictZone>& RuleEngine::conflictBlocksCanonical(
    const VehicleAgent& lo, const VehicleAgent& hi) const {
    // lo.id < hi.id(调用方保证)。按 path_gen 缓存:任一方换了固定路径才重算。
    const std::pair<int, int> key{lo.id, hi.id};
    ConflictCacheEntry& e = conflict_cache_[key];
    if (e.gen_lo != lo.path_gen || e.gen_hi != hi.path_gen) {
        e.blocks = computeConflictZonesFull(lo, hi);  // self=lo, other=hi
        e.display_overlap_polygons.clear();
        e.gen_lo = lo.path_gen;
        e.gen_hi = hi.path_gen;
    }
    return e.blocks;
}

std::vector<RuleEngine::ConflictZone> RuleEngine::findConflictZones(
    const VehicleAgent& self, const VehicleAgent& other) const {
    // 取静态 C_ij(缓存),按 self/other 朝向取用,再按当前位置裁剪——产物与历史
    // 「逐拍沿剩余路径全程精扫」在同一离散精度下等价(窗口只截低端、不会拆分连通块)。
    const bool self_is_lo = self.id < other.id;
    const VehicleAgent& lo = self_is_lo ? self : other;
    const VehicleAgent& hi = self_is_lo ? other : self;
    const std::vector<ConflictZone>& canon = conflictBlocksCanonical(lo, hi);

    // 车尾参考:车身向后伸 rear_ext。某车「已完全清出某块」= 车尾(s-rear_ext)越过该块
    // 在其路径上的出口 → 该块对它不再是冲突,丢弃。
    // 注:块的入口/出口保持「静态」(不再夹到 path_s-rear_ext)——夹紧会让上报的 se 随车
    // 前移,导致让行方的停止线(se-front)随它一起漂、永远追不上、最终蹭进区(实测 V1↔V5
    // 蹭撞的一半根因)。停止线必须是固定弧长,让行方才能稳稳停在区外。committed/cleared 仍按
    // 静态 se/sx 与当前 path_s 比较,语义不变。
    const double rear_ext = mp_.body_rear_ext();
    const double self_begin = std::max(0.0, self.path_s - rear_ext);
    const double other_begin = std::max(0.0, other.path_s - rear_ext);

    std::vector<ConflictZone> out;
    out.reserve(canon.size());
    for (const ConflictZone& cz : canon) {
        ConflictZone z = cz;
        if (!self_is_lo) {  // 朝向反转:canonical 以 lo 为 self,这里 self 是 hi
            std::swap(z.s_self_enter, z.s_other_enter);
            std::swap(z.s_self_exit, z.s_other_exit);
        }
        // 任一方已完全清出该块 → 不再冲突,丢弃。(入口不夹紧,保持静态)
        if (z.s_self_exit < self_begin || z.s_other_exit < other_begin) continue;
        out.push_back(z);
    }
    return out;
}

std::vector<ConflictMarker> RuleEngine::conflictResourceMarkers(
    const std::vector<VehicleAgent>& vehicles) const {
    std::vector<ConflictMarker> markers;
    if (!cfg_.show_prediction_conflicts) return markers;

    constexpr double kDisplayStep = 0.025;
    constexpr size_t kMaxPolygonsPerZone = 256;
    const double footprint_margin = 0.5 * cfg_.conflict_margin;

    auto buildPolygons = [&](const VehicleAgent& self,
                             const VehicleAgent& other,
                             const ConflictZone& zone) {
        // ConflictZone stores compressed arc intervals and the diagnostic
        // union AABB, but not every source polygon. Re-run the same OBB
        // predicate inside the arc rectangle for the blue display geometry;
        // non-overlapping combinations inside the compressed rectangle are
        // explicitly discarded and never rendered.
        std::map<std::pair<long long, long long>,
                 std::vector<ConflictMarker::Point>> spatial_cells;
        for (double ss = zone.s_self_enter;
             ss <= zone.s_self_exit + 1e-9; ss += kDisplayStep) {
            const double self_s = std::min(ss, zone.s_self_exit);
            const OBB self_body = makeBody(
                self.track.poseAtS(self_s), mp_, footprint_margin);
            for (double so = zone.s_other_enter;
                 so <= zone.s_other_exit + 1e-9; so += kDisplayStep) {
                const double other_s = std::min(so, zone.s_other_exit);
                const OBB other_body = makeBody(
                    other.track.poseAtS(other_s), mp_, footprint_margin);
                if (!overlaps(self_body, other_body)) continue;
                auto polygon = intersectObbs(self_body, other_body);
                if (polygon.size() < 3) continue;
                double cx = 0.0;
                double cy = 0.0;
                for (const auto& p : polygon) {
                    cx += p.x;
                    cy += p.y;
                }
                cx /= static_cast<double>(polygon.size());
                cy /= static_cast<double>(polygon.size());
                const std::pair<long long, long long> cell{
                    static_cast<long long>(std::llround(cx / kDisplayStep)),
                    static_cast<long long>(std::llround(cy / kDisplayStep))};
                spatial_cells.emplace(cell, std::move(polygon));
            }
        }

        std::vector<std::vector<ConflictMarker::Point>> polygons;
        if (spatial_cells.empty()) return polygons;
        const size_t stride = std::max<size_t>(
            1, (spatial_cells.size() + kMaxPolygonsPerZone - 1) /
                   kMaxPolygonsPerZone);
        polygons.reserve(std::min(spatial_cells.size(),
                                  kMaxPolygonsPerZone));
        size_t index = 0;
        for (auto& item : spatial_cells) {
            if (index % stride == 0 &&
                polygons.size() < kMaxPolygonsPerZone) {
                polygons.push_back(std::move(item.second));
            }
            ++index;
        }
        return polygons;
    };

    auto polygonsForZone = [&](const VehicleAgent& self,
                               const VehicleAgent& other,
                               const ConflictZone& oriented_zone)
        -> const std::vector<std::vector<ConflictMarker::Point>>& {
        const bool self_is_lo = self.id < other.id;
        const VehicleAgent& lo = self_is_lo ? self : other;
        const VehicleAgent& hi = self_is_lo ? other : self;
        const auto& canonical = conflictBlocksCanonical(lo, hi);
        ConflictCacheEntry& cache = conflict_cache_[{lo.id, hi.id}];
        if (cache.display_overlap_polygons.size() != canonical.size()) {
            cache.display_overlap_polygons.clear();
            cache.display_overlap_polygons.reserve(canonical.size());
            for (const ConflictZone& zone : canonical) {
                cache.display_overlap_polygons.push_back(
                    buildPolygons(lo, hi, zone));
            }
        }

        const double self_enter = self_is_lo
            ? oriented_zone.s_self_enter : oriented_zone.s_other_enter;
        const double self_exit = self_is_lo
            ? oriented_zone.s_self_exit : oriented_zone.s_other_exit;
        const double other_enter = self_is_lo
            ? oriented_zone.s_other_enter : oriented_zone.s_self_enter;
        const double other_exit = self_is_lo
            ? oriented_zone.s_other_exit : oriented_zone.s_self_exit;
        for (size_t index = 0; index < canonical.size(); ++index) {
            const ConflictZone& zone = canonical[index];
            if (std::abs(zone.s_self_enter - self_enter) <= 1e-9 &&
                std::abs(zone.s_self_exit - self_exit) <= 1e-9 &&
                std::abs(zone.s_other_enter - other_enter) <= 1e-9 &&
                std::abs(zone.s_other_exit - other_exit) <= 1e-9) {
                return cache.display_overlap_polygons[index];
            }
        }
        static const std::vector<std::vector<ConflictMarker::Point>> empty;
        return empty;
    };

    auto makeMarker = [&](const VehicleAgent& a, const VehicleAgent& b,
                          const ConflictZone& zone, int zone_index,
                          ConflictMarkerKind kind, int holder_id,
                          int waiter_id) {
        ConflictMarker marker;
        marker.vehicle_a = a.id;
        marker.vehicle_b = b.id;
        marker.raw_zone_index = zone.raw_index;
        marker.active_zone_index = zone_index;
        marker.path_gen_a = a.path_gen;
        marker.path_gen_b = b.path_gen;
        marker.s_a_enter = zone.s_self_enter;
        marker.s_a_exit = zone.s_self_exit;
        marker.s_b_enter = zone.s_other_enter;
        marker.s_b_exit = zone.s_other_exit;
        marker.kind = kind;
        marker.holder_id = holder_id;
        marker.waiter_id = waiter_id;
        marker.spatial_overlap_polygons = polygonsForZone(a, b, zone);

        if (zone.aabb_valid) {
            marker.x = 0.5 * (zone.aabb_min_x + zone.aabb_max_x);
            marker.y = 0.5 * (zone.aabb_min_y + zone.aabb_max_y);
            marker.scale_x = std::max(0.01, zone.aabb_max_x - zone.aabb_min_x);
            marker.scale_y = std::max(0.01, zone.aabb_max_y - zone.aabb_min_y);
            marker.zone_aabb_valid = true;
        } else {
            marker.x = zone.x;
            marker.y = zone.y;
        }
        return marker;
    };

    for (size_t i = 0; i < vehicles.size(); ++i) {
        const VehicleAgent& a = vehicles[i];
        if (!a.active() || a.track.empty()) continue;
        for (size_t j = i + 1; j < vehicles.size(); ++j) {
            const VehicleAgent& b = vehicles[j];
            if (!b.active() || b.track.empty()) continue;
            const std::vector<ConflictZone> zones = findConflictZones(a, b);
            for (size_t zone_index = 0; zone_index < zones.size();
                 ++zone_index) {
                markers.push_back(makeMarker(
                    a, b, zones[zone_index],
                    static_cast<int>(zone_index),
                    ConflictMarkerKind::POTENTIAL_CONFLICT_ZONE, -1, -1));
            }

            const std::pair<int, int> key{std::min(a.id, b.id),
                                          std::max(a.id, b.id)};
            const auto reservation = conflict_reservations_.find(key);
            if (reservation == conflict_reservations_.end()) continue;
            const ConflictReservation& r = reservation->second;
            const bool a_is_lo = a.id == key.first;
            if (r.gen_lo != (a_is_lo ? a.path_gen : b.path_gen) ||
                r.gen_hi != (a_is_lo ? b.path_gen : a.path_gen)) {
                continue;
            }
            ConflictZone reserved_zone;
            reserved_zone.s_self_enter = a_is_lo ? r.enter_lo : r.enter_hi;
            reserved_zone.s_self_exit = a_is_lo ? r.exit_lo : r.exit_hi;
            reserved_zone.s_other_enter = a_is_lo ? r.enter_hi : r.enter_lo;
            reserved_zone.s_other_exit = a_is_lo ? r.exit_hi : r.exit_lo;
            reserved_zone.x = r.x;
            reserved_zone.y = r.y;
            reserved_zone.raw_index = r.raw_zone_index;
            reserved_zone.aabb_min_x = r.aabb_min_x;
            reserved_zone.aabb_min_y = r.aabb_min_y;
            reserved_zone.aabb_max_x = r.aabb_max_x;
            reserved_zone.aabb_max_y = r.aabb_max_y;
            reserved_zone.aabb_valid = r.aabb_valid;
            int reserved_zone_index = -1;
            for (size_t zone_index = 0; zone_index < zones.size();
                 ++zone_index) {
                const ConflictZone& zone = zones[zone_index];
                if (std::abs(zone.s_self_enter -
                             reserved_zone.s_self_enter) <= 1e-9 &&
                    std::abs(zone.s_self_exit -
                             reserved_zone.s_self_exit) <= 1e-9 &&
                    std::abs(zone.s_other_enter -
                             reserved_zone.s_other_enter) <= 1e-9 &&
                    std::abs(zone.s_other_exit -
                             reserved_zone.s_other_exit) <= 1e-9) {
                    reserved_zone_index = static_cast<int>(zone_index);
                    break;
                }
            }
            const int waiter_id = r.owner_id == a.id ? b.id : a.id;
            markers.push_back(makeMarker(
                a, b, reserved_zone, reserved_zone_index,
                ConflictMarkerKind::CONFLICT_RESERVATION,
                r.owner_id, waiter_id));
        }
    }
    return markers;
}

void RuleEngine::recordConflictZones(
    const VehicleAgent& self, const VehicleAgent& other,
    const std::vector<ConflictZone>& zones, ConflictMarkerKind kind,
    double first_conflict_t, int follower_id, int leader_id,
    double following_gap, VehicleAction following_action,
    int holder_id, int waiter_id,
    const std::vector<ConflictMarker::TimedOverlap>& timed_overlaps) {
    constexpr double kDisplayStep = 0.025;
    const double following_pad =
        std::max(0.03, 0.5 * mp_.vehicle_width +
                           0.5 * cfg_.conflict_margin);

    for (const ConflictZone& z : zones) {
        double x_min = std::numeric_limits<double>::infinity();
        double y_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();

        auto includePathSpan = [&](const VehicleAgent& v,
                                   double s_enter, double s_exit) {
            const double begin = std::max(0.0, s_enter);
            const double end = std::min(v.track.length(), s_exit);
            if (end < begin) return;
            for (double s = begin; s <= end + 1e-9;
                 s += kDisplayStep) {
                const RoughWp p = v.track.poseAtS(std::min(s, end));
                x_min = std::min(x_min, p.x);
                y_min = std::min(y_min, p.y);
                x_max = std::max(x_max, p.x);
                y_max = std::max(y_max, p.y);
            }
            const RoughWp p = v.track.poseAtS(end);
            x_min = std::min(x_min, p.x);
            y_min = std::min(y_min, p.y);
            x_max = std::max(x_max, p.x);
            y_max = std::max(y_max, p.y);
        };

        if (!timed_overlaps.empty()) {
            for (const ConflictMarker::TimedOverlap& overlap : timed_overlaps) {
                for (const ConflictMarker::Point& p : overlap.polygon) {
                    x_min = std::min(x_min, p.x);
                    y_min = std::min(y_min, p.y);
                    x_max = std::max(x_max, p.x);
                    y_max = std::max(y_max, p.y);
                }
            }
        } else if (kind == ConflictMarkerKind::SAME_DIRECTION) {
            includePathSpan(self, z.s_self_enter, z.s_self_exit);
            includePathSpan(other, z.s_other_enter, z.s_other_exit);
        }
        if (!std::isfinite(x_min) || !std::isfinite(y_min) ||
            !std::isfinite(x_max) || !std::isfinite(y_max)) {
            continue;
        }

        ConflictMarker marker;
        marker.x = 0.5 * (x_min + x_max);
        marker.y = 0.5 * (y_min + y_max);
        const double pad = kind == ConflictMarkerKind::SAME_DIRECTION
            ? following_pad : 0.0;
        marker.scale_x = std::max(0.01, x_max - x_min + 2.0 * pad);
        marker.scale_y = std::max(0.01, y_max - y_min + 2.0 * pad);
        marker.vehicle_a = self.id;
        marker.vehicle_b = other.id;
        marker.path_gen_a = self.path_gen;
        marker.path_gen_b = other.path_gen;
        marker.raw_zone_index = z.raw_index;
        marker.s_a_enter = z.s_self_enter;
        marker.s_a_exit = z.s_self_exit;
        marker.s_b_enter = z.s_other_enter;
        marker.s_b_exit = z.s_other_exit;
        const auto active_zones = findConflictZones(self, other);
        for (size_t active = 0; active < active_zones.size(); ++active) {
            if (active_zones[active].raw_index == z.raw_index) {
                marker.active_zone_index = static_cast<int>(active);
                break;
            }
        }
        marker.follower_id = follower_id;
        marker.leader_id = leader_id;
        marker.holder_id = holder_id;
        marker.waiter_id = waiter_id;
        marker.following_gap = following_gap;
        marker.following_action = following_action;
        marker.timed_overlaps = timed_overlaps;
        if (follower_id >= 0 && leader_id >= 0) {
            const RoughWp follower_pose = self.track.poseAtS(self.path_s);
            const RoughWp leader_pose = other.track.poseAtS(other.path_s);
            marker.follower_x = follower_pose.x;
            marker.follower_y = follower_pose.y;
            marker.leader_x = leader_pose.x;
            marker.leader_y = leader_pose.y;
        }
        marker.kind = kind;
        marker.t = first_conflict_t;
        conflicts_.push_back(marker);
    }
}

void RuleEngine::debugDumpConflict(const VehicleAgent& a,
                                   const VehicleAgent& b) const {
    const std::vector<ConflictZone> zones = findConflictZones(a, b);
    const std::pair<int, int> pkey{std::min(a.id, b.id), std::max(a.id, b.id)};
    int owner = -1;
    auto it = conflict_reservations_.find(pkey);
    if (it != conflict_reservations_.end()) owner = it->second.owner_id;
    const bool following = following_pairs_.count(pkey) > 0;
    const double f = mp_.body_front_ext();
    bool all_same = true;
    for (const ConflictZone& z : zones)
        if (!z.same_dir) { all_same = false; break; }
    ROS_WARN("%s [CONFLICT] V%d(s=%.3f rem=%.3f act=%d blk=%d) vs "
             "V%d(s=%.3f rem=%.3f act=%d blk=%d) | owner=V%d following=%d "
             "nzones=%zu all_same_dir=%d",
             debugLogPrefix().c_str(),
             a.id, a.path_s, a.remainingS(), (int)a.action, a.blocker_id,
             b.id, b.path_s, b.remainingS(), (int)b.action, b.blocker_id,
             owner, (int)following, zones.size(), (int)all_same);
    for (size_t i = 0; i < zones.size(); ++i) {
        const ConflictZone& z = zones[i];
        ROS_WARN("%s [CONFLICT]   zone%zu same_dir=%d | A[%.3f,%.3f] stopA=%.3f "
                 "committedA=%d | B[%.3f,%.3f] stopB=%.3f committedB=%d | @(%.2f,%.2f)",
                 debugLogPrefix().c_str(), i, (int)z.same_dir,
                 z.s_self_enter, z.s_self_exit,
                 z.s_self_enter - f, (int)(a.path_s > z.s_self_enter),
                 z.s_other_enter, z.s_other_exit, z.s_other_enter - f,
                 (int)(b.path_s > z.s_other_enter), z.x, z.y);
    }
}


double RuleEngine::timeToReachS(const VehicleAgent& v, VehicleAction action,
                                 double target_s) const {
    if (target_s <= v.path_s + 1e-9) return 0.0;

    const double dist    = target_s - v.path_s;
    const double v_des   = speedForAction(action);
    const double v_cur   = std::max(0.0, v.current_speed);

    if (v_des < 1e-9) return std::numeric_limits<double>::infinity();

    // Binary search: invert predictedTravelDistance
    double lo = 0.0;
    double hi = dist / v_des * 3.0 + 2.0;
    for (int i = 0; i < 64; ++i) {
        const double mid = (lo + hi) * 0.5;
        (predictedTravelDistance(v, action, mid) < dist ? lo : hi) = mid;
    }
    return hi;
}

void RuleEngine::applyActionRequest(VehicleAgent& v, VehicleAction action,
                                    const std::string& reason,
                                    int blocker_id) {
    if (moreRestrictive(action, v.requested_action)) {
        v.requested_action = action;
        v.reason = reason;
        v.blocker_id = blocker_id;
    }
}

void RuleEngine::refreshDepartureClusterCommitments(
    std::vector<VehicleAgent>& vehicles) {
    auto agentById = [&](int id) -> VehicleAgent* {
        for (VehicleAgent& v : vehicles) {
            if (v.id == id) return &v;
        }
        return nullptr;
    };
    auto eraseWithEvent = [&](auto it, const char* event,
                              const char* reason) {
        VehicleAgent* owner = agentById(it->second.owner_id);
        VehicleAgent* other = agentById(it->second.other_id);
        logDepartureCluster(coord_log_sink_, event, reason, it->second,
                            owner ? owner->path_s : -1.0,
                            other ? other->path_s : -1.0);
        return departure_cluster_commitments_.erase(it);
    };

    for (auto it = departure_cluster_commitments_.begin();
         it != departure_cluster_commitments_.end();) {
        DepartureClusterCommitment& c = it->second;
        VehicleAgent* owner = agentById(c.owner_id);
        VehicleAgent* other = agentById(c.other_id);
        if (owner == nullptr || other == nullptr || !owner->active() ||
            !other->active() || owner->track.empty() || other->track.empty()) {
            it = eraseWithEvent(it, c.active ? "INVALIDATE" : "INVALIDATE",
                                "vehicle_or_path_invalid");
            continue;
        }

        if (!c.active) {
            if (owner->mission_phase == MissionPhase::TO_B &&
                owner->path_gen == c.owner_path_gen &&
                other->path_gen == c.other_path_gen &&
                other->mission_phase == MissionPhase::TO_A1) {
                c.active = true;
                c.handoff_already_inside = false;
                for (const auto& z : c.intervals) {
                    if (other->path_s > z.other_enter + 1e-9 &&
                        other->path_s <= z.other_exit + 1e-9) {
                        c.handoff_already_inside = true;
                        break;
                    }
                }
                logDepartureCluster(
                    coord_log_sink_, "CREATE",
                    c.handoff_already_inside ? "handoff_already_inside"
                                             : "future_handoff",
                    c, owner->path_s, other->path_s);
                ++it;
                continue;
            }
            const bool preview_still_matches =
                (owner->mission_phase == MissionPhase::TO_A1 ||
                 owner->mission_phase == MissionPhase::PICKUP_DWELL) &&
                owner->pending_dropoff_valid &&
                !owner->pending_dropoff_track.empty() &&
                owner->path_gen + 1 == c.owner_path_gen &&
                other->path_gen == c.other_path_gen &&
                other->mission_phase == MissionPhase::TO_A1 &&
                (owner->mission_phase == MissionPhase::PICKUP_DWELL ||
                 (future_a1_commitment_.valid() &&
                  future_a1_commitment_.owner_id == owner->id &&
                  future_a1_commitment_.owner_path_gen == owner->path_gen));
            if (!preview_still_matches) {
                it = eraseWithEvent(it, "INVALIDATE", "staged_handoff_invalid");
            } else {
                ++it;
            }
            continue;
        }

        if (!departureClusterGenerationsMatch(
                c.owner_path_gen, owner->path_gen,
                c.other_path_gen, other->path_gen)) {
            it = eraseWithEvent(it, "INVALIDATE", "path_gen_changed");
        } else if (owner->mission_phase != MissionPhase::TO_B ||
                   other->mission_phase != MissionPhase::TO_A1) {
            it = eraseWithEvent(it, "INVALIDATE", "mission_phase_changed");
        } else if (departureClusterCleared(
                       owner->path_s, c.owner_release_exit_s,
                       other->path_s, c.other_release_exit_s)) {
            const char* reason = owner->path_s > c.owner_release_exit_s + 1e-9
                ? "owner_cleared_cluster" : "other_cleared_cluster";
            it = eraseWithEvent(it, "RELEASE", reason);
        } else {
            ++it;
        }
    }

    // Fallback for a TO_B activation that was not represented in the current
    // Future snapshot. It deterministically rebuilds the same protected
    // closure from the actual prepared track and the existing full-zone cache.
    constexpr double kStopBuffer = 0.01;
    for (VehicleAgent& owner : vehicles) {
        if (!owner.active() || owner.track.empty() ||
            owner.mission_phase != MissionPhase::TO_B ||
            !owner.a1_departure_committed ||
            owner.a1_departure_priority_until_s <= 1e-9) {
            continue;
        }
        for (VehicleAgent& other : vehicles) {
            if (other.id == owner.id || !other.active() || other.track.empty() ||
                other.mission_phase != MissionPhase::TO_A1) {
                continue;
            }
            const std::pair<int, int> key{std::min(owner.id, other.id),
                                          std::max(owner.id, other.id)};
            if (departure_cluster_commitments_.count(key) != 0) continue;
            const bool owner_is_lo = owner.id < other.id;
            const VehicleAgent& lo = owner_is_lo ? owner : other;
            const VehicleAgent& hi = owner_is_lo ? other : owner;
            const auto& blocks = conflictBlocksCanonical(lo, hi);
            const FutureA1ZoneSelection selected =
                selectFutureA1ProtectedZones(
                    blocks, owner_is_lo,
                    owner.a1_departure_priority_until_s, other.path_s);
            if (selected.protected_indices.empty() ||
                selected.upstream_index < 0) {
                continue;
            }
            DepartureClusterCommitment c;
            c.owner_id = owner.id;
            c.owner_path_gen = owner.path_gen;
            c.other_id = other.id;
            c.other_path_gen = other.path_gen;
            c.seed_indices = selected.seed_indices;
            c.cluster_indices = selected.protected_indices;
            c.waiter_stop_boundary_s =
                selected.normalized_zones[static_cast<size_t>(
                    selected.upstream_index)].s_other_enter;
            c.waiter_stop_s =
                std::max(0.0, c.waiter_stop_boundary_s - kStopBuffer);
            c.active = true;
            c.handoff_already_inside = selected.other_already_inside;
            for (size_t index : selected.protected_indices) {
                const ConflictZone& z = selected.normalized_zones[index];
                c.intervals.push_back(FutureA1ConflictInterval{
                    z.s_self_enter, z.s_self_exit,
                    z.s_other_enter, z.s_other_exit});
                c.owner_release_exit_s =
                    std::max(c.owner_release_exit_s, z.s_self_exit);
                c.other_release_exit_s =
                    std::max(c.other_release_exit_s, z.s_other_exit);
            }
            departure_cluster_commitments_[key] = c;
            logDepartureCluster(
                coord_log_sink_, "CREATE",
                c.handoff_already_inside ? "handoff_already_inside"
                                         : "deterministic_rebuild",
                c, owner.path_s, other.path_s);
        }
    }
}

int RuleEngine::departureClusterOwnerForPair(const VehicleAgent& a,
                                             const VehicleAgent& b) const {
    const std::pair<int, int> key{std::min(a.id, b.id), std::max(a.id, b.id)};
    const auto it = departure_cluster_commitments_.find(key);
    if (it == departure_cluster_commitments_.end() || !it->second.active) {
        return -1;
    }
    const DepartureClusterCommitment& c = it->second;
    const VehicleAgent* owner = a.id == c.owner_id ? &a :
                                b.id == c.owner_id ? &b : nullptr;
    const VehicleAgent* other = a.id == c.other_id ? &a :
                                b.id == c.other_id ? &b : nullptr;
    if (owner == nullptr || other == nullptr ||
        owner->path_gen != c.owner_path_gen ||
        other->path_gen != c.other_path_gen) {
        return -1;
    }
    if (futureA1OtherInsideCluster(c.intervals, other->path_s)) {
        return other->id;  // Actual occupancy is stronger than handoff.
    }
    return owner->id;
}

void RuleEngine::enforceDepartureClusterCommitments(
    std::vector<VehicleAgent>& vehicles, double dt) {
    auto agentById = [&](int id) -> VehicleAgent* {
        for (VehicleAgent& v : vehicles) {
            if (v.id == id) return &v;
        }
        return nullptr;
    };
    for (auto& entry : departure_cluster_commitments_) {
        DepartureClusterCommitment& c = entry.second;
        if (!c.active) continue;
        VehicleAgent* owner = agentById(c.owner_id);
        VehicleAgent* other = agentById(c.other_id);
        if (owner == nullptr || other == nullptr) continue;
        const bool already_inside =
            futureA1OtherInsideCluster(c.intervals, other->path_s);
        if (already_inside) continue;

        const double distance = c.waiter_stop_s - other->path_s;
        const double speed = std::max(0.0, other->current_speed);
        const double stopping_distance =
            speed * speed / (2.0 * std::max(1e-6, cfg_.max_decel)) +
            speed * dt;
        if (distance > stopping_distance + 1e-9) continue;
        applyActionRequest(*other, VehicleAction::STOP,
                           "departure_cluster_priority", owner->id);
        if (!c.hold_logged) {
            logDepartureCluster(coord_log_sink_, "HOLD",
                                "cluster_stop_boundary", c,
                                owner->path_s, other->path_s);
            c.hold_logged = true;
        }
    }
}


PairInteractionResult RuleEngine::detectPairInteraction(
    const VehicleAgent& a, const VehicleAgent& b,
    double prediction_horizon) const {
    const std::vector<ConflictZone> zones = findConflictZones(a, b);
    const auto prediction_a =
        predictTrajectory(a, mp_, cfg_, VehicleAction::NOMINAL,
                          prediction_horizon);
    const auto prediction_b =
        predictTrajectory(b, mp_, cfg_, VehicleAction::NOMINAL,
                          prediction_horizon);
    return detectPairInteractionFromPredictions(
        a, b, zones, prediction_a, prediction_b);
}

void RuleEngine::resolvePairwiseConflicts(std::vector<VehicleAgent>& vehicles,
                                          double dt,
                                          double prediction_horizon) {
    pairwise_managed_pairs_.clear();
    // Predict each vehicle once. Pair detection below is pure and consumes
    // these shared samples without changing coordination state.
    const double horizon =
        std::max(cfg_.prediction_step, prediction_horizon);
    const double prediction_step = std::max(0.02, cfg_.prediction_step);
    std::vector<std::vector<PredictedKinematicSample>> predictions(
        vehicles.size());
    for (size_t i = 0; i < vehicles.size(); ++i) {
        if (vehicles[i].active() && !vehicles[i].track.empty()) {
            predictions[i] =
                predictTrajectory(vehicles[i], mp_, cfg_,
                                  VehicleAction::NOMINAL, horizon);
        }
    }

    auto agentById = [&](int id) -> VehicleAgent* {
        for (VehicleAgent& v : vehicles) {
            if (v.id == id) return &v;
        }
        return nullptr;
    };
    auto terminalDocking = [&](const VehicleAgent& v) {
        if (!v.active()) return false;
        const double terminal_distance =
            std::max(cfg_.target_request_distance, cfg_.target_stop_distance);
        return v.remainingS() <= terminal_distance;
    };

    // ConflictZone arc intervals already represent rear-axle reference poses
    // whose inflated complete-body OBBs overlap. Do not expand them by another
    // vehicle length when deciding entry, clearance, or the stop line.
    constexpr double kStopBuffer = 0.01;
    auto insideInterval = [](const VehicleAgent& v, double enter_s,
                             double exit_s) {
        return v.path_s > enter_s + 1e-9 &&
               v.path_s <= exit_s + 1e-9;
    };
    auto brakeBefore = [&](VehicleAgent& v, double conflict_enter_s,
                           int other_id) {
        if (v.deadlock_breaker) return;
        const double stop_s = std::max(0.0, conflict_enter_s - kStopBuffer);
        const double distance = stop_s - v.path_s;
        const double speed = std::max(0.0, v.current_speed);
        const double stopping_distance =
            speed * speed / (2.0 * std::max(1e-6, cfg_.max_decel)) +
            speed * dt;
        if (distance <= stopping_distance + 1e-9) {
            applyActionRequest(v, VehicleAction::STOP,
                               "time_brake_V" + std::to_string(other_id),
                               other_id);
        }
    };
    const bool dynamic_speed_enabled = vehicles.size() == 2;
    auto logNominalRecovery = [&](const std::pair<int, int>& key) {
        if (!dynamic_speed_enabled) return;
        const auto previous = previous_dynamic_actions_.find(key);
        if (previous == previous_dynamic_actions_.end()) return;
        ++dynamic_speed_metrics_.nominal_recoveries;
        if (!coord_log_sink_) return;
        std::ostringstream line;
        line << "[DYN-SPEED] pair=V" << key.first << "-V" << key.second
             << " baseline=NOMINAL/NOMINAL baseline_conflict=false"
             << " previous_action=" << actionName(previous->second)
             << " selected=NOMINAL/NOMINAL"
             << " reason=rolling_recovery";
        coord_log_sink_(line.str());
    };

    // Drop event reservations when a participant or its route changes.
    for (auto it = conflict_reservations_.begin();
         it != conflict_reservations_.end();) {
        VehicleAgent* lo = agentById(it->first.first);
        VehicleAgent* hi = agentById(it->first.second);
        const ConflictReservation& r = it->second;
        if (lo == nullptr || hi == nullptr || !lo->active() || !hi->active() ||
            lo->path_gen != r.gen_lo || hi->path_gen != r.gen_hi) {
            logConflictReservation(coord_log_sink_, it->first, "delete", r);
            ++dynamic_speed_metrics_.reservation_deletes;
            it = conflict_reservations_.erase(it);
        } else {
            ++it;
        }
    }

    auto reservationZone = [](const ConflictReservation& r, bool a_is_lo) {
        ConflictZone z;
        z.s_self_enter = a_is_lo ? r.enter_lo : r.enter_hi;
        z.s_self_exit = a_is_lo ? r.exit_lo : r.exit_hi;
        z.s_other_enter = a_is_lo ? r.enter_hi : r.enter_lo;
        z.s_other_exit = a_is_lo ? r.exit_hi : r.exit_lo;
        z.x = r.x;
        z.y = r.y;
        z.raw_index = r.raw_zone_index;
        z.aabb_min_x = r.aabb_min_x;
        z.aabb_min_y = r.aabb_min_y;
        z.aabb_max_x = r.aabb_max_x;
        z.aabb_max_y = r.aabb_max_y;
        z.aabb_valid = r.aabb_valid;
        return z;
    };

    auto futureA1OwnerForPair = [&](const VehicleAgent& a,
                                    const VehicleAgent& b) {
        if (!future_a1_commitment_.valid() ||
            a.mission_phase != MissionPhase::TO_A1 ||
            b.mission_phase != MissionPhase::TO_A1) {
            return -1;
        }
        const VehicleAgent* owner = nullptr;
        const VehicleAgent* other = nullptr;
        if (a.id == future_a1_commitment_.owner_id &&
            a.path_gen == future_a1_commitment_.owner_path_gen) {
            owner = &a;
            other = &b;
        } else if (b.id == future_a1_commitment_.owner_id &&
                   b.path_gen == future_a1_commitment_.owner_path_gen) {
            owner = &b;
            other = &a;
        }
        if (owner == nullptr || !owner->pending_dropoff_valid ||
            owner->pending_dropoff_track.empty() ||
            owner->a1_departure_priority_until_s <= 1e-9) {
            return -1;
        }

        VehicleAgent exit_preview = *owner;
        exit_preview.track = owner->pending_dropoff_track;
        exit_preview.path_s = 0.0;
        exit_preview.path_gen = owner->path_gen + 1;
        exit_preview.mode = VehicleMode::ACTIVE;
        exit_preview.mission_phase = MissionPhase::TO_B;

        const bool preview_is_lo = exit_preview.id < other->id;
        const VehicleAgent& lo = preview_is_lo ? exit_preview : *other;
        const VehicleAgent& hi = preview_is_lo ? *other : exit_preview;
        const std::pair<int, int> cache_key{lo.id, hi.id};
        ConflictCacheEntry& cache = future_a1_conflict_cache_[cache_key];
        if (cache.gen_lo != lo.path_gen || cache.gen_hi != hi.path_gen) {
            cache.blocks = computeConflictZonesFull(lo, hi);
            cache.gen_lo = lo.path_gen;
            cache.gen_hi = hi.path_gen;
        }

        const FutureA1ZoneSelection future_zones =
            selectFutureA1ProtectedZones(
                cache.blocks, preview_is_lo,
                owner->a1_departure_priority_until_s, other->path_s);
        if (future_zones.protected_indices.empty() ||
            future_zones.other_already_inside) {
            return -1;  // Actual occupancy remains stronger.
        }

        // The other vehicle may be outside the prepared exit conflict but
        // already physically committed to a conflict on the owner's current
        // TO_A1 leg. Actual occupancy must remain stronger than the future
        // commitment in that case as well.
        const bool owner_is_lo = owner->id < other->id;
        const VehicleAgent& ordinary_lo = owner_is_lo ? *owner : *other;
        const VehicleAgent& ordinary_hi = owner_is_lo ? *other : *owner;
        const auto& ordinary_blocks =
            conflictBlocksCanonical(ordinary_lo, ordinary_hi);
        for (const ConflictZone& canonical : ordinary_blocks) {
            const double owner_exit = owner_is_lo
                ? canonical.s_self_exit : canonical.s_other_exit;
            const double other_enter = owner_is_lo
                ? canonical.s_other_enter : canonical.s_self_enter;
            const double other_exit = owner_is_lo
                ? canonical.s_other_exit : canonical.s_self_exit;
            if (owner->path_s > owner_exit + 1e-9 ||
                other->path_s > other_exit + 1e-9) {
                continue;
            }
            if (other->path_s > other_enter + 1e-9) {
                return -1;
            }
        }
        return owner->id;
    };

    for (size_t i = 0; i < vehicles.size(); ++i) {
        VehicleAgent& a = vehicles[i];
        if (!a.active() || predictions[i].empty()) continue;
        for (size_t j = i + 1; j < vehicles.size(); ++j) {
            VehicleAgent& b = vehicles[j];
            if (!b.active() || predictions[j].empty()) continue;

            const std::pair<int, int> key{std::min(a.id, b.id),
                                          std::max(a.id, b.id)};
            const bool a_is_lo = a.id == key.first;
            const std::vector<ConflictZone> zones = findConflictZones(a, b);
            if (zones.empty()) {
                logNominalRecovery(key);
                const auto old = conflict_reservations_.find(key);
                if (old != conflict_reservations_.end()) {
                    logConflictReservation(coord_log_sink_, key, "delete",
                                           old->second);
                    ++dynamic_speed_metrics_.reservation_deletes;
                    conflict_reservations_.erase(old);
                }
                continue;
            }
            const PairInteractionResult interaction =
                detectPairInteractionFromPredictions(
                    a, b, zones, predictions[i], predictions[j]);

            // Honor the previously selected local event, not every later
            // crossing of the same pair of complete paths.
            auto reservation_it = conflict_reservations_.find(key);
            if (reservation_it != conflict_reservations_.end()) {
                if (dynamic_speed_enabled) {
                    ++dynamic_speed_metrics_.existing_reservation_skips;
                    if (coord_log_sink_) {
                        std::ostringstream line;
                        line << "[DYN-SPEED] pair=V" << key.first << "-V"
                             << key.second
                             << " candidate_search=SKIPPED"
                             << " reason=existing_reservation"
                             << " fallback=legacy";
                        coord_log_sink_(line.str());
                    }
                }
                ConflictReservation& r = reservation_it->second;
                VehicleAgent& lo = a_is_lo ? a : b;
                VehicleAgent& hi = a_is_lo ? b : a;

                // A horizon-scoped Future A1 owner may replace only a soft
                // forecast holder. Keep the reservation itself and never
                // override a vehicle already physically inside this event.
                const bool lo_inside =
                    insideInterval(lo, r.enter_lo, r.exit_lo);
                const bool hi_inside =
                    insideInterval(hi, r.enter_hi, r.exit_hi);
                const int departure_cluster_owner =
                    departureClusterOwnerForPair(a, b);
                const int future_owner = futureA1OwnerForPair(a, b);
                const int protected_owner = departure_cluster_owner >= 0
                    ? departure_cluster_owner : future_owner;
                if (!lo_inside && !hi_inside && protected_owner >= 0 &&
                    r.owner_id != protected_owner) {
                    r.owner_id = protected_owner;
                    logConflictReservation(coord_log_sink_, key, "update", r);
                }

                VehicleAgent& owner = r.owner_id == lo.id ? lo : hi;
                VehicleAgent& waiter = r.owner_id == lo.id ? hi : lo;
                const double owner_enter = r.owner_id == lo.id
                    ? r.enter_lo : r.enter_hi;
                const double owner_exit = r.owner_id == lo.id
                    ? r.exit_lo : r.exit_hi;
                const double waiter_enter = r.owner_id == lo.id
                    ? r.enter_hi : r.enter_lo;
                const double waiter_exit = r.owner_id == lo.id
                    ? r.exit_hi : r.exit_lo;

                if (owner.path_s > owner_exit + 1e-9) {
                    logConflictReservation(coord_log_sink_, key, "delete", r);
                    ++dynamic_speed_metrics_.reservation_deletes;
                    conflict_reservations_.erase(reservation_it);
                } else {
                    const std::vector<ConflictMarker::TimedOverlap>
                        reserved_overlaps = decimateTimedOverlaps(
                            interaction.event.timed_overlaps);
                    pairwise_managed_pairs_.insert(key);
                    const bool owner_inside =
                        insideInterval(owner, owner_enter, owner_exit);
                    const bool waiter_inside =
                        insideInterval(waiter, waiter_enter, waiter_exit);
                    if (waiter_inside && !owner_inside &&
                        owner.path_s <= owner_enter + 1e-9) {
                        // Current physical occupancy overrides an old forecast.
                        r.owner_id = waiter.id;
                        logConflictReservation(coord_log_sink_, key, "update", r);
                        brakeBefore(owner, owner_enter, waiter.id);
                        if (owner.reason ==
                            "time_brake_V" + std::to_string(waiter.id)) {
                            if (shouldLogA1Decision(owner, waiter.id)) {
                                logA1Decision(coord_log_sink_, cfg_, owner,
                                              &waiter, waiter.id);
                            }
                        }
                    } else {
                        brakeBefore(waiter, waiter_enter, owner.id);
                        if (waiter.reason ==
                            "time_brake_V" + std::to_string(owner.id)) {
                            if (shouldLogA1Decision(waiter, owner.id)) {
                                logA1Decision(coord_log_sink_, cfg_, waiter,
                                              &owner, owner.id);
                            }
                        }
                    }
                    const ConflictZone rz = reservationZone(r, a_is_lo);
                    recordConflictZones(
                        a, b, std::vector<ConflictZone>{rz},
                        ConflictMarkerKind::CROSSING_OR_OPPOSING,
                        r.first_conflict_t, -1, -1, 0.0,
                        VehicleAction::NOMINAL, r.owner_id, waiter.id,
                        reserved_overlaps);
                    continue;
                }
            }

            const TimedConflictEvent& event = interaction.event;
            if (!event.valid) {
                logNominalRecovery(key);
                continue;
            }

            pairwise_managed_pairs_.insert(key);

            const ConflictZone& zone =
                zones[static_cast<size_t>(event.associated_zone_index)];
            const bool a_inside =
                insideInterval(a, zone.s_self_enter, zone.s_self_exit);
            const bool b_inside =
                insideInterval(b, zone.s_other_enter, zone.s_other_exit);
            bool a_inside_any_zone = false;
            bool b_inside_any_zone = false;
            for (const ConflictZone& candidate_zone : zones) {
                a_inside_any_zone = a_inside_any_zone || insideInterval(
                    a, candidate_zone.s_self_enter,
                    candidate_zone.s_self_exit);
                b_inside_any_zone = b_inside_any_zone || insideInterval(
                    b, candidate_zone.s_other_enter,
                    candidate_zone.s_other_exit);
            }
            int departure_cluster_owner = -1;
            int future_owner = -1;
            const bool a_a1_departure =
                a.a1_departure_committed &&
                a.path_s < a.a1_departure_priority_until_s - 1e-9;
            const bool b_a1_departure =
                b.a1_departure_committed &&
                b.path_s < b.a1_departure_priority_until_s - 1e-9;
            if (!a_inside && !b_inside) {
                departure_cluster_owner =
                    departureClusterOwnerForPair(a, b);
                future_owner = futureA1OwnerForPair(a, b);
            }
            const bool future_a1_pair =
                future_a1_commitment_.valid() &&
                ((a.id == future_a1_commitment_.owner_id &&
                  a.path_gen == future_a1_commitment_.owner_path_gen) ||
                 (b.id == future_a1_commitment_.owner_id &&
                  b.path_gen == future_a1_commitment_.owner_path_gen));
            const bool a1_related =
                departure_cluster_commitments_.count(key) != 0 ||
                future_a1_pair || a_a1_departure || b_a1_departure ||
                departure_cluster_owner >= 0 || future_owner >= 0;
            const bool a_terminal = terminalDocking(a);
            const bool b_terminal = terminalDocking(b);

            if (dynamic_speed_enabled) {
                ++dynamic_speed_metrics_.baseline_conflicts;
                const bool ordinary =
                    !a_inside_any_zone && !b_inside_any_zone && !a1_related &&
                    !a_terminal && !b_terminal &&
                    !a.deadlock_breaker && !b.deadlock_breaker;
                const int preferred_winner = ordinary
                    ? priorityWinner(a, b) : -1;
                bool near_fallback = false;
                if (ordinary && preferred_winner >= 0) {
                    const VehicleAgent& yielding_vehicle =
                        preferred_winner == a.id ? b : a;
                    const double yielding_entry =
                        preferred_winner == a.id
                            ? zone.s_other_enter : zone.s_self_enter;
                    near_fallback = hasInsufficientBrakingMargin(
                        yielding_vehicle, yielding_entry, cfg_, dt,
                        kStopBuffer);
                }

                PairSpeedCoordinationResult speed_result;
                if (ordinary && !near_fallback) {
                    speed_result = evaluatePairSpeedCoordination(
                        a, b, zones, interaction, mp_, cfg_, horizon,
                        preferred_winner);
                    for (const SpeedCoordinationCandidate& candidate :
                         speed_result.candidates) {
                        const bool yield_trial =
                            candidate.action_a == VehicleAction::YIELD ||
                            candidate.action_b == VehicleAction::YIELD;
                        if (yield_trial) {
                            ++dynamic_speed_metrics_.yield_trials;
                            if (candidate.conflict_free) {
                                ++dynamic_speed_metrics_.yield_clear;
                            }
                        } else {
                            ++dynamic_speed_metrics_.creep_trials;
                            if (candidate.conflict_free) {
                                ++dynamic_speed_metrics_.creep_clear;
                            }
                        }
                    }
                    if (speed_result.fallback_required) {
                        ++dynamic_speed_metrics_.candidate_search_failed;
                    }
                } else if (near_fallback) {
                    ++dynamic_speed_metrics_.near_fallbacks;
                } else if (a1_related) {
                    ++dynamic_speed_metrics_.a1_fallbacks;
                }

                if (coord_log_sink_) {
                    std::ostringstream line;
                    line << std::fixed << std::setprecision(3)
                         << "[DYN-SPEED] pair=V" << key.first << "-V"
                         << key.second
                         << " baseline=NOMINAL/NOMINAL"
                         << " baseline_conflict=true"
                         << " baseline_first_t=" << event.first_t;
                    for (const SpeedCoordinationCandidate& candidate :
                         speed_result.candidates) {
                        line << " try=" << actionName(candidate.action_a)
                             << "/" << actionName(candidate.action_b)
                             << ":"
                             << (candidate.conflict_free
                                     ? "CLEAR" : "CONFLICT");
                        if (candidate.first_conflict_t) {
                            line << "@" << *candidate.first_conflict_t;
                        }
                    }
                    if (speed_result.solved_by_speed_adjustment) {
                        line << " selected="
                             << actionName(speed_result.selected_action_a)
                             << "/"
                             << actionName(speed_result.selected_action_b)
                             << " reason=" << speed_result.reason
                             << " reservation=not_created";
                    } else {
                        const char* skip_reason = near_fallback
                            ? "insufficient_braking_margin"
                            : (a1_related ? "a1_protected"
                               : (!ordinary ? "legacy_special_case"
                                  : speed_result.reason.c_str()));
                        line << " candidate_search="
                             << (ordinary && !near_fallback
                                     ? "FAILED" : "SKIPPED")
                             << " reason=" << skip_reason
                             << " fallback=legacy";
                    }
                    coord_log_sink_(line.str());
                }

                if (speed_result.solved_by_speed_adjustment) {
                    const std::string suffix =
                        "_V" + std::to_string(preferred_winner);
                    if (speed_result.selected_action_a !=
                        VehicleAction::NOMINAL) {
                        applyActionRequest(
                            a, speed_result.selected_action_a,
                            "dynamic_speed_" + std::string(actionName(
                                speed_result.selected_action_a)) + suffix,
                            b.id);
                    }
                    if (speed_result.selected_action_b !=
                        VehicleAction::NOMINAL) {
                        applyActionRequest(
                            b, speed_result.selected_action_b,
                            "dynamic_speed_" + std::string(actionName(
                                speed_result.selected_action_b)) + suffix,
                            a.id);
                    }
                    recordConflictZones(
                        a, b, std::vector<ConflictZone>{zone},
                        ConflictMarkerKind::CROSSING_OR_OPPOSING,
                        event.first_t, -1, -1, 0.0,
                        VehicleAction::NOMINAL, preferred_winner,
                        preferred_winner == a.id ? b.id : a.id,
                        decimateTimedOverlaps(event.timed_overlaps));
                    continue;
                }
            }

            int holder = -1;
            if (a_inside != b_inside) {
                holder = a_inside ? a.id : b.id;
            } else if (a_inside && b_inside) {
                // If both are already committed, clear the one that can leave
                // this local event sooner instead of creating a double stop.
                const double a_clear = timeToReachS(
                    a, VehicleAction::NOMINAL, zone.s_self_exit);
                const double b_clear = timeToReachS(
                    b, VehicleAction::NOMINAL, zone.s_other_exit);
                if (std::abs(a_clear - b_clear) > prediction_step) {
                    holder = a_clear < b_clear ? a.id : b.id;
                } else {
                    holder = priorityWinner(a, b);
                }
            } else {
                if (departure_cluster_owner >= 0) {
                    holder = departure_cluster_owner;
                } else if (future_owner >= 0) {
                    holder = future_owner;
                } else if (a_a1_departure != b_a1_departure) {
                    // Normal A1 case: neither vehicle has entered this event,
                    // so the prepared pickup departure owns only this visible
                    // spatiotemporal conflict. An already-inside vehicle is
                    // handled by the branch above and is never forced to obey
                    // an impossible late stop.
                    holder = a_a1_departure ? a.id : b.id;
                } else {
                    if (a_terminal != b_terminal) {
                        holder = a_terminal ? a.id : b.id;
                    } else {
                        holder = priorityWinner(a, b);
                    }
                }
            }

            const std::tuple<int, int, int> conflict_log_key{
                key.first, key.second, holder};
            if (coord_log_sink_ &&
                pairwise_conflict_logs_.insert(conflict_log_key).second) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(3)
                     << "[PAIRWISE_CONFLICT] component=PAIRWISE pair=V"
                     << key.first << "-V" << key.second
                     << " holder="
                     << (holder >= 0 ? "V" + std::to_string(holder) : "none")
                     << " waiter=";
                if (holder == a.id) {
                    line << "V" << b.id;
                } else if (holder == b.id) {
                    line << "V" << a.id;
                } else {
                    line << "both";
                }
                line << " conflict_type=CROSSING/OPPOSING"
                     << " first_t=" << event.first_t;
                coord_log_sink_(line.str());
            }

            recordConflictZones(
                a, b, std::vector<ConflictZone>{zone},
                ConflictMarkerKind::CROSSING_OR_OPPOSING, event.first_t,
                -1, -1, 0.0, VehicleAction::NOMINAL, holder,
                holder == a.id ? b.id : (holder == b.id ? a.id : -1),
                decimateTimedOverlaps(event.timed_overlaps));
            if (holder < 0) {
                brakeBefore(a, zone.s_self_enter, b.id);
                brakeBefore(b, zone.s_other_enter, a.id);
                if (a.reason == "time_brake_V" + std::to_string(b.id)) {
                    if (shouldLogA1Decision(a, b.id)) {
                        logA1Decision(coord_log_sink_, cfg_, a, &b, b.id);
                    }
                }
                if (b.reason == "time_brake_V" + std::to_string(a.id)) {
                    if (shouldLogA1Decision(b, a.id)) {
                        logA1Decision(coord_log_sink_, cfg_, b, &a, a.id);
                    }
                }
                continue;
            }

            ConflictReservation r;
            r.owner_id = holder;
            r.gen_lo = a_is_lo ? a.path_gen : b.path_gen;
            r.gen_hi = a_is_lo ? b.path_gen : a.path_gen;
            r.enter_lo = a_is_lo ? zone.s_self_enter : zone.s_other_enter;
            r.exit_lo = a_is_lo ? zone.s_self_exit : zone.s_other_exit;
            r.enter_hi = a_is_lo ? zone.s_other_enter : zone.s_self_enter;
            r.exit_hi = a_is_lo ? zone.s_other_exit : zone.s_self_exit;
            r.x = zone.x;
            r.y = zone.y;
            r.first_conflict_t = event.first_t;
            r.raw_zone_index = zone.raw_index;
            r.aabb_min_x = zone.aabb_min_x;
            r.aabb_min_y = zone.aabb_min_y;
            r.aabb_max_x = zone.aabb_max_x;
            r.aabb_max_y = zone.aabb_max_y;
            r.aabb_valid = zone.aabb_valid;
            conflict_reservations_[key] = r;
            ++dynamic_speed_metrics_.reservation_creates;
            logConflictReservation(coord_log_sink_, key, "create", r);

            if (holder == a.id) {
                brakeBefore(b, zone.s_other_enter, a.id);
                if (b.reason == "time_brake_V" + std::to_string(a.id)) {
                    if (shouldLogA1Decision(b, a.id)) {
                        logA1Decision(coord_log_sink_, cfg_, b, &a, a.id);
                    }
                }
            } else {
                brakeBefore(a, zone.s_self_enter, b.id);
                if (a.reason == "time_brake_V" + std::to_string(b.id)) {
                    if (shouldLogA1Decision(a, b.id)) {
                        logA1Decision(coord_log_sink_, cfg_, a, &b, b.id);
                    }
                }
            }
        }
    }
}

void RuleEngine::enforceFutureA1Admission(
    std::vector<VehicleAgent>& vehicles, double dt) {
    if (!future_a1_commitment_.valid()) return;

    VehicleAgent* owner = nullptr;
    for (VehicleAgent& v : vehicles) {
        if (v.id == future_a1_commitment_.owner_id) {
            owner = &v;
            break;
        }
    }
    if (owner == nullptr || owner->path_gen !=
                                future_a1_commitment_.owner_path_gen ||
        !owner->pending_dropoff_valid ||
        owner->pending_dropoff_track.empty() ||
        (owner->mission_phase != MissionPhase::TO_A1 &&
         owner->mission_phase != MissionPhase::PICKUP_DWELL)) {
        return;
    }

    // Compare the already-prepared A1->B exit against each other vehicle's
    // current TO_A1 path. The synthetic generation is exactly the generation
    // activatePreparedDropoffLeg() will assign to this frozen exit track.
    VehicleAgent exit_preview = *owner;
    exit_preview.track = owner->pending_dropoff_track;
    exit_preview.path_s = 0.0;
    exit_preview.path_gen = owner->path_gen + 1;
    exit_preview.mode = VehicleMode::ACTIVE;
    exit_preview.mission_phase = MissionPhase::TO_B;

    const double protected_until = owner->a1_departure_priority_until_s;
    if (protected_until <= 1e-9) return;
    constexpr double kStopBuffer = 0.01;

    for (VehicleAgent& other : vehicles) {
        if (other.id == owner->id || !other.active() ||
            other.mission_phase != MissionPhase::TO_A1 ||
            other.track.empty()) {
            continue;
        }

        const bool preview_is_lo = exit_preview.id < other.id;
        const VehicleAgent& lo = preview_is_lo ? exit_preview : other;
        const VehicleAgent& hi = preview_is_lo ? other : exit_preview;
        const std::pair<int, int> key{lo.id, hi.id};
        ConflictCacheEntry& cache = future_a1_conflict_cache_[key];
        if (cache.gen_lo != lo.path_gen || cache.gen_hi != hi.path_gen) {
            cache.blocks = computeConflictZonesFull(lo, hi);
            cache.gen_lo = lo.path_gen;
            cache.gen_hi = hi.path_gen;
        }

        const FutureA1ZoneSelection future_zones =
            selectFutureA1ProtectedZones(cache.blocks, preview_is_lo,
                                         protected_until, other.path_s);
        std::optional<double> future_exit_enter_s;
        ConflictZone future_selected;
        if (future_zones.upstream_index >= 0) {
            future_selected = future_zones.normalized_zones[
                static_cast<size_t>(future_zones.upstream_index)];
            future_exit_enter_s = future_selected.s_other_enter;
        }

        // Admission must also keep the non-owner upstream of any still-relevant
        // conflict on the owner's current TO_A1 leg. These are the same static
        // OBB conflict intervals used by ordinary pairwise arbitration.
        bool ordinary_already_inside = false;
        std::optional<double> ordinary_enter_s;
        ConflictZone ordinary_selected;
        const bool owner_is_lo = owner->id < other.id;
        const VehicleAgent& ordinary_lo = owner_is_lo ? *owner : other;
        const VehicleAgent& ordinary_hi = owner_is_lo ? other : *owner;
        const auto& ordinary_blocks =
            conflictBlocksCanonical(ordinary_lo, ordinary_hi);
        for (const ConflictZone& canonical : ordinary_blocks) {
            ConflictZone zone = canonical;
            if (!owner_is_lo) {
                std::swap(zone.s_self_enter, zone.s_other_enter);
                std::swap(zone.s_self_exit, zone.s_other_exit);
            }
            // Ignore conflict blocks already cleared by either participant.
            if (owner->path_s > zone.s_self_exit + 1e-9 ||
                other.path_s > zone.s_other_exit + 1e-9) {
                continue;
            }
            if (other.path_s > zone.s_other_enter + 1e-9) {
                ordinary_already_inside = true;
                ordinary_enter_s = zone.s_other_enter;
                ordinary_selected = zone;
                break;
            }
            if (!ordinary_enter_s ||
                zone.s_other_enter < *ordinary_enter_s) {
                ordinary_enter_s = zone.s_other_enter;
                ordinary_selected = zone;
            }
        }

        // A future exit conflict is what makes this pair subject to Future A1
        // admission. Ordinary geometry only moves its stop line upstream.
        if (!future_exit_enter_s) continue;

        const bool already_inside =
            future_zones.other_already_inside || ordinary_already_inside;
        const std::optional<double> selected_stop_boundary_s =
            futureA1StopBoundary(future_exit_enter_s, ordinary_enter_s);
        const std::optional<double> selected_stop_s =
            futureA1StopS(future_exit_enter_s, ordinary_enter_s, kStopBuffer);
        const bool ordinary_selected_boundary =
            ordinary_enter_s && selected_stop_boundary_s &&
            std::abs(*ordinary_enter_s - *selected_stop_boundary_s) <= 1e-9;
        const ConflictZone& selected = ordinary_selected_boundary
            ? ordinary_selected : future_selected;

        // Stage the exact admission boundary and transitive exit cluster for
        // the generation activatePreparedDropoffLeg() will assign. The entry
        // remains inert until refreshDepartureClusterCommitments() observes
        // the actual TO_B transition, so it cannot act as a second Future
        // owner during TO_A1/PICKUP_DWELL.
        const std::pair<int, int> cluster_key{
            std::min(owner->id, other.id), std::max(owner->id, other.id)};
        auto existing_cluster =
            departure_cluster_commitments_.find(cluster_key);
        if (existing_cluster == departure_cluster_commitments_.end() ||
            !existing_cluster->second.active) {
            DepartureClusterCommitment staged;
            staged.owner_id = owner->id;
            staged.owner_path_gen = exit_preview.path_gen;
            staged.other_id = other.id;
            staged.other_path_gen = other.path_gen;
            staged.seed_indices = future_zones.seed_indices;
            staged.cluster_indices = future_zones.protected_indices;
            staged.waiter_stop_boundary_s = *selected_stop_boundary_s;
            staged.waiter_stop_s = *selected_stop_s;
            staged.handed_off_from_future = true;
            staged.handoff_already_inside = already_inside;
            for (size_t index : future_zones.protected_indices) {
                const ConflictZone& z = future_zones.normalized_zones[index];
                staged.intervals.push_back(FutureA1ConflictInterval{
                    z.s_self_enter, z.s_self_exit,
                    z.s_other_enter, z.s_other_exit});
                staged.owner_release_exit_s =
                    std::max(staged.owner_release_exit_s, z.s_self_exit);
                staged.other_release_exit_s =
                    std::max(staged.other_release_exit_s, z.s_other_exit);
            }
            departure_cluster_commitments_[cluster_key] = std::move(staged);
        }

        auto appendAdmissionGeometry = [&](std::ostringstream& line) {
            line << " future_exit_enter_s=" << *future_exit_enter_s
                 << " ordinary_enter_s=";
            if (ordinary_enter_s) line << *ordinary_enter_s;
            else line << "none";
            line << " selected_stop_boundary_s="
                 << *selected_stop_boundary_s
                 << " stop_s=" << *selected_stop_s
                 << " other_s=" << other.path_s
                 << " already_inside="
                 << (already_inside ? "true" : "false")
                 << " seed_zones=[";
            for (size_t i = 0; i < future_zones.seed_indices.size(); ++i) {
                if (i > 0) line << ",";
                line << future_zones.seed_indices[i];
            }
            line << "] closure_zones=[";
            for (size_t i = 0;
                 i < future_zones.protected_indices.size(); ++i) {
                if (i > 0) line << ",";
                line << future_zones.protected_indices[i];
            }
            line << "] selected_zone_count="
                 << future_zones.protected_indices.size()
                 << " inclusion_reason="
                 << (future_zones.protected_indices.size() >
                             future_zones.seed_indices.size()
                         ? "protected_seed+other_interval_overlap"
                         : "protected_seed");
        };

        const std::pair<int, int> log_key{owner->id, other.id};
        if (already_inside) {
            if (future_a1_admission_logged_.insert(log_key).second) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(3)
                     << "[FUTURE_A1_ADMISSION] owner=V" << owner->id
                     << " blocked=V" << other.id
                     << " reason=actual_occupied_priority"
                     << " early_stop=false"
                     << " holder=V" << other.id
                     << " conflict_zone=(" << selected.x << ","
                     << selected.y << ")";
                appendAdmissionGeometry(line);
                if (coord_log_sink_) coord_log_sink_(line.str());
                ROS_WARN("%s %s", debugLogPrefix().c_str(),
                         line.str().c_str());
            }
            continue;
        }
        const double stop_s = *selected_stop_s;
        const double distance = stop_s - other.path_s;
        const double speed = std::max(0.0, other.current_speed);
        const double stopping_distance =
            speed * speed / (2.0 * std::max(1e-6, cfg_.max_decel)) +
            speed * dt;
        if (distance > stopping_distance + 1e-9) continue;

        applyActionRequest(other, VehicleAction::STOP,
                           "future_a1_exit_priority", owner->id);
        if (future_a1_admission_logged_.insert(log_key).second) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[FUTURE_A1_ADMISSION] owner=V" << owner->id
                 << " blocked=V" << other.id
                 << " reason=future_a1_exit_priority"
                 << " early_stop=true"
                 << " holder=V" << owner->id
                 << " conflict_zone=(" << selected.x << "," << selected.y
                 << ")";
            appendAdmissionGeometry(line);
            if (coord_log_sink_) coord_log_sink_(line.str());
            ROS_WARN("%s %s", debugLogPrefix().c_str(),
                     line.str().c_str());
        }
    }
}

void RuleEngine::resolveFollowing(std::vector<VehicleAgent>& vehicles) {
    // 跟车只识别唯一的纵向 leader/follower 并产生低优先级建议。
    // 它不再跳过 timed OBB、不删除 reservation、不参与 holder 选择。
    following_pairs_.clear();
    following_suggestions_.clear();
    std::set<std::pair<int, int>> ambiguous_following_pairs;
    auto motionHeading = [](const VehicleAgent& v) {
        constexpr double kPi = 3.14159265358979323846;
        double heading = v.track.poseAtS(v.path_s).theta;
        if (v.track.typeAtS(v.path_s) == WpType::REVERSE) {
            heading += kPi;
        }
        return heading;
    };

    auto terminalDocking = [&](const VehicleAgent& v) {
        if (!v.active()) return false;
        const double terminal_distance =
            std::max(cfg_.target_request_distance, cfg_.target_stop_distance);
        return v.remainingS() <= terminal_distance;
    };

    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        if (terminalDocking(v)) continue;

        const RoughWp pose_v = v.track.poseAtS(v.path_s);
        const double heading_v = motionHeading(v);

        for (const VehicleAgent& other : vehicles) {
            if (other.id == v.id || !other.active()) continue;
            const RoughWp pose_o = other.track.poseAtS(other.path_s);
            const double heading_o = motionHeading(other);
            const double current_dir_dot =
                std::cos(heading_v) * std::cos(heading_o) +
                std::sin(heading_v) * std::sin(heading_o);
            // Static zone direction is not sufficient near curves/cusps.
            // Require the current physical motion directions (REVERSE already
            // converted by motionHeading) to agree as well.
            if (current_dir_dot <= 0.70) continue;
            // 静态冲突块方向仅作为附加几何条件；当前真实运动方向已在上方
            // 独立校验，避免块中点方向与当前局部方向不一致。
            const std::vector<ConflictZone> fzones = findConflictZones(v, other);
            if (fzones.empty()) continue;
            bool all_same_dir = true;
            for (const ConflictZone& z : fzones) {
                if (!z.same_dir) { all_same_dir = false; break; }
            }
            if (!all_same_dir) continue;

            const double dx = pose_o.x - pose_v.x;
            const double dy = pose_o.y - pose_v.y;
            const double fwd = dx * std::cos(heading_v) +
                               dy * std::sin(heading_v);
            if (fwd <= 0.0) continue;

            const double lat = std::abs(-dx * std::sin(heading_v) +
                                         dy * std::cos(heading_v));
            if (lat > mp_.vehicle_width) continue;

            // 确认 v 在同向局部车道上跟随 other。此集合只用于诊断，
            // resolvePairwiseConflicts 仍会完整执行。
            const std::pair<int, int> key{std::min(v.id, other.id),
                                          std::max(v.id, other.id)};
            if (ambiguous_following_pairs.count(key) != 0) continue;
            if (following_pairs_.count(key) != 0) {
                // Both directed scans claimed to be the follower. Cancel the
                // relation and leave this pair to timed OBB arbitration.
                following_pairs_.erase(key);
                following_suggestions_.erase(key);
                ambiguous_following_pairs.insert(key);
                continue;
            }
            following_pairs_.insert(key);

            const double dist = std::hypot(dx, dy);
            const double gap = dist - mp_.vehicle_length;

            VehicleAction follow_action;
            if (gap <= cfg_.following_min_distance) {
                follow_action = VehicleAction::STOP;
            } else if (gap <= cfg_.following_creep_distance) {
                follow_action = VehicleAction::CREEP;
            } else if (gap <= cfg_.following_normal_distance) {
                follow_action = VehicleAction::YIELD;
            } else {
                continue;
            }
            recordConflictZones(v, other, fzones,
                                ConflictMarkerKind::SAME_DIRECTION, 0.0,
                                v.id, other.id, gap, follow_action);
            following_suggestions_[key] = FollowingSuggestion{
                v.id, other.id, follow_action, gap};
        }
    }
}

void RuleEngine::applyFollowingSuggestions(
    std::vector<VehicleAgent>& vehicles) {
    for (const auto& previous : previous_following_followers_) {
        if (pairwise_managed_pairs_.count(previous.first) == 0) continue;
        for (VehicleAgent& v : vehicles) {
            if (v.id != previous.second || !v.active()) continue;
            if (v.requested_action == VehicleAction::NOMINAL) {
                // Pairwise selected this former follower as the unblocked
                // side. Do not let the old low-priority following action_hold
                // negate that holder decision. Motion still ramps through the
                // normal acceleration limit in the vehicle advance step.
                v.action = VehicleAction::NOMINAL;
                v.action_hold_remaining = 0.0;
            }
            break;
        }
    }

    for (const auto& item : following_suggestions_) {
        if (pairwise_managed_pairs_.count(item.first) != 0) continue;
        const FollowingSuggestion& suggestion = item.second;
        for (VehicleAgent& v : vehicles) {
            if (v.id != suggestion.follower_id || !v.active()) continue;
            // Any earlier arbitration/safety request wins, regardless of
            // action severity. The normal merge is restrictive-only and
            // cannot otherwise express a low-priority STOP suggestion.
            if (v.requested_action != VehicleAction::NOMINAL) break;
            applyActionRequest(
                v, suggestion.action,
                "following_V" + std::to_string(suggestion.leader_id),
                suggestion.leader_id);
            break;
        }
    }
}

void RuleEngine::enforceForwardClearance(std::vector<VehicleAgent>& vehicles) {
    // 普适前向净空护栏(§11.13.1 出口检查精神 + 补 following/crossing 分类接缝漏洞)。
    // 接缝 bug:近乎同向、向不同库位汇聚的两车,被 pairwise 当跟车跳过、又不满足
    // resolveFollowing 的横向/间距条件 → 两套都没刹 → 后车 NOMINAL 直撞停在路口的前车
    // → 形成谁前进都撞对方的十字楔死(canStepForward 双 false、破环无效、硬护栏反复)。
    // 兜底:任何车沿自身固定路径在「自己刹车距离 + 车头前伸」内会压上另一辆车的当前
    // 车身,就提前 STOP。不论被哪套逻辑处理这道都在。比硬护栏(0 余量、贴死才停)早刹、
    // 留余量 → 两车干净对停而非重叠 → 破环车 canStepForward 能判出谁可走 → 解开。
    const double front_ext = mp_.body_front_ext();
    auto bodyAt = [&](const VehicleAgent& v, double s) {
        return makeBody(v.track.poseAtS(s), mp_, 0.0);
    };
    auto currentS = [&](const VehicleAgent& v) {
        return (v.mode == VehicleMode::DWELL) ? v.track.length() : v.path_s;
    };
    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        if (v.deadlock_breaker) continue;  // 破环车豁免:它正被授权冲出环
        if (v.requested_action == VehicleAction::STOP) continue;
        const double v_cur = std::max(0.0, v.current_speed);
        const double brake_dist =
            (v_cur * v_cur) / (2.0 * std::max(1e-6, cfg_.max_decel));
        // 前探距离必须足够远,让车「早早停在冲突区外、留出间隙」,而不是冲到贴上才刹
        // (低速时 brake_dist 极小,只算它会一直蹭到接触才停=楔死)。故在刹车距离之外
        // 再加:车头前伸 + 一个固定安全间隙 kStandoff。kStandoff 同时是「干净对停」后
        // 两车之间留出的余量,使破环车 canStepForward 有空间判别谁能动。另设最小前探
        // kMinLook,保证即便停着(brake_dist≈0)也能看到近处已挡在交叉口的车。
        constexpr double kStandoff = 0.16;  // 停在冲突区外留出的安全间隙
        constexpr double kMinLook = 0.22;   // 最小前探(覆盖交叉接近段,防停车时漏看)
        const double look =
            std::max(brake_dist + kStandoff, kMinLook) + front_ext;
        const double s_end = std::min(v.track.length(), v.path_s + look);
        constexpr double kStep = 0.03;
        int block_id = -1;
        for (double s = v.path_s; s <= s_end + 1e-9 && block_id < 0; s += kStep) {
            const OBB body = bodyAt(v, std::min(s, s_end));
            for (const VehicleAgent& o : vehicles) {
                if (o.id == v.id) continue;
                if (o.mode == VehicleMode::NEED_TASK || o.track.empty()) continue;
                if (overlaps(body, bodyAt(o, currentS(o)))) {
                    block_id = o.id;
                    break;
                }
            }
        }
        if (block_id >= 0) {
            applyActionRequest(v, VehicleAction::STOP,
                               "clear_block_V" + std::to_string(block_id),
                               block_id);
            if (v.reason == "clear_block_V" + std::to_string(block_id)) {
                const VehicleAgent* blocker = nullptr;
                for (const VehicleAgent& candidate : vehicles) {
                    if (candidate.id == block_id) {
                        blocker = &candidate;
                        break;
                    }
                }
                if (shouldLogA1Decision(v, block_id)) {
                    logA1Decision(coord_log_sink_, cfg_, v, blocker, block_id);
                }
            }
        }
    }
}

void RuleEngine::resolveTargetSlotOccupancy(
    std::vector<VehicleAgent>& vehicles) {
    constexpr double kMouthWait = 0.35;  // hold this far short of the target
    constexpr double kSlotClear = 0.30;  // occupant must travel this far to free it

    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        const double dist_to_mouth = v.remainingS() - kMouthWait;
        const double v_cur = std::max(0.0, v.current_speed);
        const double braking_dist =
            (v_cur * v_cur) / (2.0 * std::max(1e-6, cfg_.max_decel));
        if (dist_to_mouth > braking_dist + 0.06) continue;

        for (const VehicleAgent& o : vehicles) {
            if (o.id == v.id) continue;
            if (o.current_slot != v.target_slot) continue;  // not at v's slot
            const bool occupying =
                (o.mode == VehicleMode::DWELL) ||
                (o.active() && o.path_s < kSlotClear);
            if (!occupying) continue;
            applyActionRequest(v, VehicleAction::STOP,
                               "wait_slot_V" + std::to_string(o.id), o.id);
            break;
        }
    }
}

void RuleEngine::applyRequestedActions(std::vector<VehicleAgent>& vehicles,
                                       double dt) {
    const double hold = cfg_.action_hold_time;
    for (VehicleAgent& v : vehicles) {
        if (v.mode != VehicleMode::ACTIVE) {
            v.action = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.action_hold_remaining = 0.0;
            continue;
        }

        // 死锁打破豁免倒计时：豁免期内不允许规则层将车辆降为 STOP
        if (v.cycle_break_immunity > 0.0) {
            v.cycle_break_immunity = std::max(0.0, v.cycle_break_immunity - dt);
        }

        const VehicleAction prev = v.action;            // last cycle's output
        VehicleAction req = v.requested_action;         // this cycle's rules

        // 死锁豁免：仅当 blocker 确实是死锁环成员（等待链最终指回 v）
        // 时才降为 CREEP；若 blocker 只是恰好停着等第三方（非死锁），
        // 安全第一，保持 STOP，不能朝停着的车 CREEP 过去。
        if (v.cycle_break_immunity > 0.0 && req == VehicleAction::STOP) {
            bool blocker_in_cycle = false;
            if (v.blocker_id >= 0) {
                int cur_id = v.blocker_id;
                const int kMaxHops = static_cast<int>(vehicles.size()) + 1;
                for (int hop = 0; hop < kMaxHops && cur_id >= 0; ++hop) {
                    if (cur_id == v.id) { blocker_in_cycle = true; break; }
                    int next_id = -1;
                    for (const VehicleAgent& o : vehicles) {
                        if (o.id == cur_id) {
                            if (o.action == VehicleAction::STOP && o.blocker_id >= 0)
                                next_id = o.blocker_id;
                            break;
                        }
                    }
                    cur_id = next_id;
                }
            }
            if (blocker_in_cycle) {
                req = VehicleAction::CREEP;  // 验证为死锁环，缓行打破
            } else {
                // blocker 不是死锁环成员，安全第一
                v.cycle_break_immunity = 0.0;
            }
        }

        // §9 破环车保底:即便某层仍要它 STOP,也强制至少 CREEP 冲出环(它已在资源/
        // 优先级层拿到最高优先级,这里保证动作落地)。硬护栏仍兜底防真碰撞。
        if (v.deadlock_breaker && req == VehicleAction::STOP) {
            req = VehicleAction::CREEP;
            v.reason = "deadlock_break";
        }

        if (hold <= 0.0) {                              // smoothing disabled
            v.action = req;
            continue;
        }

        if (moreRestrictive(req, prev)) {
            v.action = req;
            v.action_hold_remaining = hold;
        } else if (req == prev) {
            v.action = prev;
            v.action_hold_remaining =
                std::max(0.0, v.action_hold_remaining - dt);
        } else {
            v.action_hold_remaining -= dt;
            if (v.action_hold_remaining > 0.0) {
                v.action = prev;
                if (v.reason == "clear") v.reason = "action_hold";
            } else {
                v.action = minAction(relaxOneStep(prev), req);
                v.action_hold_remaining = hold;
            }
        }
    }
}

void RuleEngine::breakDeadlockCycles(std::vector<VehicleAgent>& vehicles) {
    const size_t n = vehicles.size();

    auto indexOfId = [&](int id) -> int {
        for (size_t i = 0; i < n; ++i) {
            if (vehicles[i].id == id) return static_cast<int>(i);
        }
        return -1;
    };
    auto waitEdge = [&](int i) -> int {
        const VehicleAgent& v = vehicles[i];
        if (v.mode != VehicleMode::ACTIVE) return -1;
        if (v.action != VehicleAction::STOP) return -1;  // only fully-stopped waits
        if (v.blocker_id < 0) return -1;
        return indexOfId(v.blocker_id);
    };

    std::vector<int> visit_state(n, 0);  // 0 unvisited, 1 in-progress, 2 done
    for (size_t s = 0; s < n; ++s) {
        if (visit_state[s] != 0) continue;
        std::vector<int> path;
        int cur = static_cast<int>(s);
        while (cur >= 0 && visit_state[cur] == 0) {
            visit_state[cur] = 1;
            path.push_back(cur);
            cur = waitEdge(cur);
        }
        if (cur >= 0 && visit_state[cur] == 1) {
            const auto cycle_begin = std::find(path.begin(), path.end(), cur);
            int release = -1;
            int release_id = std::numeric_limits<int>::max();
            for (auto p = cycle_begin; p != path.end(); ++p) {
                if (vehicles[*p].id < release_id) {
                    release_id = vehicles[*p].id;
                    release = *p;
                }
            }
            if (release >= 0) {
                VehicleAgent& r = vehicles[release];
                r.action = VehicleAction::YIELD;
                r.requested_action = VehicleAction::YIELD;
                r.action_hold_remaining = 0.0;
                r.cycle_break_immunity = 0.6;  // 0.6s 内规则层不得将该车重新降为 STOP
                r.wait_time = 0.0;
                r.reason = "cycle_break";
            }
        }
        for (int p : path) visit_state[p] = 2;
    }
}

void RuleEngine::arbitrateResources(std::vector<VehicleAgent>& vehicles,
                                    double dt) {
    if (resmap_ == nullptr) return;
    const double front = mp_.body_front_ext();
    const double rear = mp_.body_rear_ext();
    const double decel = std::max(1e-6, cfg_.max_decel);
    const double nominal = speedForAction(VehicleAction::NOMINAL);

    // 资源申请窗口(§15 第二类:不要提前老远占用资源)。只有当车已逼近资源到
    // 「必须开始决策能否在停止线前停住」的预警点时才请求/预约;离得远的资源不预约,
    // 以免一辆还在老远的高优先级车把资源锁死、让近处车无谓干等("明明没啥事就不走")。
    // 窗口 = 标称速刹停距离 + 2 个车身(决策缓冲),纯几何派生,非拍脑袋常数。
    const double request_window =
        nominal * nominal / (2.0 * decel) + 2.0 * mp_.vehicle_length;

    // 1) 按资源聚合请求者:active 车的固定路径会用到该 capacity=1 互斥资源
    //    (窄道/路口/货位口)、车尾还没整车驶出、且已进入申请窗口。
    struct Req { size_t idx; double s_enter; double s_exit; };
    std::map<int, std::vector<Req>> by_res;
    for (size_t i = 0; i < vehicles.size(); ++i) {
        const VehicleAgent& v = vehicles[i];
        if (!v.active()) continue;
        for (const ResourceSpan& sp : v.resource_spans) {
            const TrafficResource* r = resmap_->byId(sp.resource_id);
            if (r == nullptr || r->capacity != 1) continue;
            if (r->type != ResourceType::NARROW &&
                r->type != ResourceType::INTERSECTION &&
                r->type != ResourceType::SLOT_DOCK) {
                continue;
            }
            if (v.path_s - rear > sp.s_exit + 1e-6) continue;  // 已整车驶出
            if (sp.s_enter - v.path_s > request_window) continue;  // 尚远,不预约
            by_res[sp.resource_id].push_back(Req{i, sp.s_enter, sp.s_exit});
        }
    }

    auto bodyInside = [&](const VehicleAgent& v, const Req& q) {
        return v.path_s + front >= q.s_enter - 1e-6 &&
               v.path_s - rear <= q.s_exit + 1e-6;
    };

    // 某车当前车身(后轴还原车身中心)。DWELL 用终点位姿,空轨迹跳过。
    auto poseOf = [&](const VehicleAgent& o) {
        const double s = (o.mode == VehicleMode::DWELL) ? o.track.length()
                                                        : o.path_s;
        return o.track.poseAtS(std::min(s, o.track.length()));
    };
    // 出口检查(§11.4/§11.13.1):候选车驶出资源后的落脚处(s_exit + 半车长)
    // 是否被别的车身占住。被占 → 进去就会卡在资源里 → 不发令牌(除非它已在区内
    // 必须驶完)。这杜绝"令牌发给进得去出不来的车、它攥着令牌却动不了挡死所有人"。
    auto canExit = [&](size_t vi, const Req& q) {
        const VehicleAgent& v = vehicles[vi];
        const double len = v.track.length();
        const double s_check = std::min(q.s_exit + 0.5 * mp_.vehicle_length, len);
        const OBB body = makeBody(v.track.poseAtS(s_check), mp_, 0.0);
        for (size_t o = 0; o < vehicles.size(); ++o) {
            if (o == vi) continue;
            const VehicleAgent& ov = vehicles[o];
            if (ov.mode != VehicleMode::ACTIVE && ov.mode != VehicleMode::DWELL)
                continue;
            if (ov.track.empty()) continue;
            if (overlaps(body, makeBody(poseOf(ov), mp_, 0.0))) return false;
        }
        return true;
    };
    // 可授予 = 已在区内(必须驶完)或 出口畅通。
    auto grantableK = [&](const std::vector<Req>& rs, size_t k) {
        return bodyInside(vehicles[rs[k].idx], rs[k]) || canExit(rs[k].idx, rs[k]);
    };

    // 2) 逐资源仲裁:已持令牌且仍在请求 → 保持(持权到整车驶出,防翻转,§11);
    //    否则按统一优先级 PriorityKey 选 winner 并发令牌(§11.2)。
    for (auto& kv : by_res) {
        const int rid = kv.first;
        std::vector<Req>& reqs = kv.second;

        // 持有者保持令牌的前提:它仍在请求 且 仍可授予(能驶出/已在区内)。
        // 在「可授予」候选里按统一 PriorityKey 选 winner(§11.2)。持令牌者带
        // already_has_token 加成(防翻转,§11),破环车带 emergency 加成(临时最高,§9)
        // —— 于是 破环车 > 持令牌者 > 其他,一套优先级统一裁决,不再用 holder 捷径。
        const int holder = tokens_.holder(rid);
        int winner_k = -1;
        {
            ResourceRequest best;
            bool has_best = false;
            for (size_t k = 0; k < reqs.size(); ++k) {
                if (!grantableK(reqs, k)) continue;  // 出口被堵的不参与(进去会卡死)
                const VehicleAgent& v = vehicles[reqs[k].idx];
                ResourceRequest rq;
                rq.vehicle_id = v.id;
                rq.wait_time = v.wait_time;
                rq.loaded = v.loaded;
                rq.task_count = v.task_count;
                rq.already_inside = bodyInside(v, reqs[k]);
                rq.already_has_token = (v.id == holder);
                rq.emergency_or_clear = v.deadlock_breaker;  // §9 破环车临时最高
                rq.starving = v.wait_time > cfg_.starvation_wait_time;
                rq.eta = rq.already_inside
                             ? 0.0
                             : timeToReachS(v, VehicleAction::NOMINAL,
                                            std::max(0.0, reqs[k].s_enter - front));
                if (!has_best || PriorityKey::betterThan(rq, best)) {
                    best = rq;
                    has_best = true;
                    winner_k = static_cast<int>(k);
                }
            }
        }
        // 无人可授予(都出不去)→ 释放令牌,且下面让所有逼近者停在上游(谁都别进)。
        const int winner_id = (winner_k >= 0) ? vehicles[reqs[winner_k].idx].id : -1;
        if (winner_id >= 0) {
            tokens_.grant(rid, winner_id, now_);  // 授予/刷新(防超时)
        } else {
            tokens_.release(rid);
        }

        // 3) 非 winner(及无人可授予时的全部逼近者):在上游停止线让行。只在「按当前
        //    速度+max_decel 即将刹不住」时才发 STOP(§10 停止线;§15 不过早等——离得
        //    远就继续接近,不原地干等)。
        for (size_t k = 0; k < reqs.size(); ++k) {
            if (winner_k >= 0 && static_cast<int>(k) == winner_k) continue;
            VehicleAgent& v = vehicles[reqs[k].idx];
            const double stop_line = reqs[k].s_enter - front;
            const double dist = stop_line - v.path_s;
            const double vc = std::max(0.0, v.current_speed);
            const double brake = vc * vc / (2.0 * decel) + vc * dt;
            if (dist <= brake + 1e-9) {
                const std::string why = (winner_id >= 0)
                    ? ("res_wait_V" + std::to_string(winner_id))
                    : "res_exit_blocked";
                applyActionRequest(v, VehicleAction::STOP, why, winner_id);
            }
        }
    }

    tokens_.expireStale(now_, std::max(2.0, cfg_.prediction_horizon));
}

void RuleEngine::refreshResourceSpans(std::vector<VehicleAgent>& vehicles) {
    if (resmap_ == nullptr) return;
    for (VehicleAgent& v : vehicles) {
        if (v.track.empty()) {
            if (!v.resource_spans.empty()) v.resource_spans.clear();
            v.spans_track_len = -1.0;
            continue;
        }
        // track 长度变化即视为换了新路径(新任务),重算资源占用区间。
        if (std::abs(v.track.length() - v.spans_track_len) > 1e-6) {
            v.resource_spans = resmap_->spansForPath(v.track);
            v.spans_track_len = v.track.length();
        }
    }
}

void RuleEngine::resolveDeadlock(std::vector<VehicleAgent>& vehicles, double dt) {
    // §9/§11.11:用上一周期残留的等待边(blocker_id,本周期 reset 前仍有效)建等待图,
    // 检测环,选破环车给临时最高优先级(deadlock_breaker)。等待图是功能图(每车至多
    // 一条出边=它在等的 blocker),跟指针即可找环。
    constexpr double kBreakerHold = 2.0;  // 破环身份迟滞保持秒数(防闪烁蹭行)
    const size_t n = vehicles.size();
    // 迟滞:保持期未到的破环车继续是破环车(它一动起来就不再 STOP 等待、环检测会消失,
    // 若不保持就会标志闪烁→被旧层反复摁停→蹭行)。保持期到了才清。
    for (VehicleAgent& v : vehicles) {
        v.deadlock_breaker_hold = std::max(0.0, v.deadlock_breaker_hold - dt);
        v.deadlock_breaker = (v.deadlock_breaker_hold > 0.0);
    }

    auto indexOfId = [&](int id) -> int {
        for (size_t i = 0; i < n; ++i)
            if (vehicles[i].id == id) return static_cast<int>(i);
        return -1;
    };
    // 破环车必须选「真正能往前走的那辆」(§16):用车身几何实测——车 i 沿固定路径
    // 前进一小步(probe),车身是否仍与任何其它车的当前车身不重叠。能=它前进可脱困、
    // 抖开环;不能(前方就是对冲车)=放它也是被硬护栏摁死、环永远破不了。这修正了
    // 旧的 min-id 盲选:楔死时常把「动不了的那辆」选成破环车而徒劳。
    auto bodyAtCurrent = [&](const VehicleAgent& v) {
        const double s =
            (v.mode == VehicleMode::DWELL) ? v.track.length() : v.path_s;
        return makeBody(v.track.poseAtS(s), mp_, 0.0);
    };
    auto canStepForward = [&](int i) -> bool {
        const VehicleAgent& v = vehicles[i];
        if (v.track.empty()) return false;
        constexpr double kProbe = 0.10;  // 前探约半身,足以判别前方是否被对冲车堵死
        const double s_next = std::min(v.track.length(), v.path_s + kProbe);
        const OBB body_next = makeBody(v.track.poseAtS(s_next), mp_, 0.0);
        for (size_t j = 0; j < n; ++j) {
            if (static_cast<int>(j) == i) continue;
            const VehicleAgent& o = vehicles[j];
            if (o.track.empty() || o.mode == VehicleMode::NEED_TASK) continue;
            if (overlaps(body_next, bodyAtCurrent(o))) return false;
        }
        return true;
    };
    auto waitEdge = [&](int i) -> int {
        const VehicleAgent& v = vehicles[i];
        if (v.mode != VehicleMode::ACTIVE) return -1;
        if (v.action != VehicleAction::STOP) return -1;  // 只看完全停住的等待
        if (v.blocker_id < 0) return -1;
        return indexOfId(v.blocker_id);
    };

    std::vector<int> state(n, 0);  // 0 未访 1 在栈 2 完成
    for (size_t s = 0; s < n; ++s) {
        if (state[s] != 0) continue;
        std::vector<int> path;
        int cur = static_cast<int>(s);
        while (cur >= 0 && state[cur] == 0) {
            state[cur] = 1;
            path.push_back(cur);
            cur = waitEdge(cur);
        }
        if (cur >= 0 && state[cur] == 1) {  // 找到环:从 cur 首次出现到末尾
            const auto begin = std::find(path.begin(), path.end(), cur);
            // 协调图第5步:严格全序(unifiedPriority)使"谁让谁"关系本应无环 ⇒ 这里检测到
            // 环=无环保证被破坏的 bug 信号(通常是 priorityWinner 里的 slot 资源前置约束
            // 成环,即任务分配出现循环占位)。降为断言:**告警**把它暴露出来。破环逃生暂留
            // 作安全网(确认长跑不再告警后可移除)。
            {
                std::string ring;
                for (auto p = begin; p != path.end(); ++p) {
                    if (p != begin) ring += "->";
                    ring += "V" + std::to_string(vehicles[*p].id);
                }
                ROS_WARN_THROTTLE(
                    5.0,
                    "[DIAG cycle] 等待图检测到环 %s —— 严格全序下本不该出现,"
                    "疑为 slot 资源前置约束成环(循环占位)。暂由破环逃生兜底。",
                    ring.c_str());
            }
            // 选破环车:优先「前进一步能脱困」的车(canStepForward),同类里取 id 最小
            // (确定性,§19)。若环内无人能前进(真·楔死,需倒车的罕见情形),退回 id
            // 最小。只放一辆:其余正常让行停住——绝不会两辆对冲车一起被放而相撞。
            int breaker = -1, best_id = std::numeric_limits<int>::max();
            int breaker_any = -1, best_any = std::numeric_limits<int>::max();
            for (auto p = begin; p != path.end(); ++p) {
                const int vid = vehicles[*p].id;
                if (vid < best_any) { best_any = vid; breaker_any = *p; }
                if (canStepForward(*p) && vid < best_id) {
                    best_id = vid;
                    breaker = *p;
                }
            }
            if (breaker < 0) breaker = breaker_any;  // 无人能前进→退回 min-id
            // 破环逃生已停用(降为纯检测+告警):在「禁止倒车」前提下,前向破环本身是碰撞源
            // ——它豁免让行方刹车、强推它前冲脱困,而前方就是环里别的车 → 直接顶上去(实测
            // V2 brkr=1 被强推顶进卡死的 V7,此后两车贴死、硬护栏每拍触发=那 16857 次"碰撞")。
            // 既然不能倒车,环就应是「静止对峙(楔死)」而非「对撞」:安全优先,先把碰撞降级。
            // 真正出路是从源头不让环形成(出库口/出口检查),见 草履虫规则_协调图统一架构设计。
            (void)breaker; (void)kBreakerHold;
            // vehicles[breaker].deadlock_breaker = true;          // 已停用
            // vehicles[breaker].deadlock_breaker_hold = kBreakerHold;
        }
        for (int p : path) state[p] = 2;
    }
}

void RuleEngine::decide(std::vector<VehicleAgent>& vehicles, double dt,
                        double prediction_horizon_override) {
    conflicts_.clear();
    now_ += dt;                      // 内部仿真时钟(令牌防抖/超时)
    resolveDeadlock(vehicles, dt);   // Phase4:用上周期等待边检测环、选破环车(reset 前)
    refreshResourceSpans(vehicles);  // Phase 2:刷新每车路径的资源占用缓存

    previous_following_followers_.clear();
    previous_dynamic_actions_.clear();
    for (const VehicleAgent& v : vehicles) {
        if (v.blocker_id < 0) continue;
        if (v.reason.rfind("dynamic_speed_", 0) == 0 &&
            (v.action == VehicleAction::YIELD ||
             v.action == VehicleAction::CREEP)) {
            const std::pair<int, int> key{
                std::min(v.id, v.blocker_id),
                std::max(v.id, v.blocker_id)};
            previous_dynamic_actions_[key] = v.action;
        }
        const std::string expected =
            "following_V" + std::to_string(v.blocker_id);
        if (v.reason != expected) continue;
        const std::pair<int, int> key{std::min(v.id, v.blocker_id),
                                      std::max(v.id, v.blocker_id)};
        previous_following_followers_[key] = v.id;
    }

    for (VehicleAgent& v : vehicles) {
        v.blocker_id = -1;
        if (v.mode != VehicleMode::ACTIVE) {
            v.cycle_break_immunity = 0.0;
            v.requested_action = VehicleAction::STOP;
            v.reason = "not_active";
            continue;
        }
        v.requested_action = VehicleAction::NOMINAL;
        v.reason = "clear";
    }

    resolveFollowing(vehicles);
    // 深层根治(用户洞察:路径固定→只信精确几何):停用粗粒度资源盒仲裁(路口/车道
    // 令牌),它把"共用一个路口盒"当冲突造成幻象冲突→打架→死锁。改由精确的 pairwise
    // 几何冲突(findConflictZones:沿固定路径采样车身OBB,只标真实重叠弧段)作唯一交叉
    // 协调权威。八竿子打不着的两车它根本不报冲突→各自全速。
    // arbitrateResources(vehicles, dt);   // 已停用(资源盒=幻象冲突源)
    const double pairwise_horizon = prediction_horizon_override >= 0.0
        ? prediction_horizon_override
        : cfg_.prediction_horizon;
    refreshDepartureClusterCommitments(vehicles);
    resolvePairwiseConflicts(vehicles, dt, pairwise_horizon);
    enforceFutureA1Admission(vehicles, dt);
    enforceDepartureClusterCommitments(vehicles, dt);
    resolveTargetSlotOccupancy(vehicles);  // slot-mouth queueing (spec 6/7)
    enforceForwardClearance(vehicles);     // 普适前向净空兜底:堵分类接缝→防十字楔死
    applyFollowingSuggestions(vehicles);   // lowest-priority longitudinal hint
    applyRequestedActions(vehicles, dt);
    if (cfg_.enable_cycle_break) breakDeadlockCycles(vehicles);  // spec 16

    for (VehicleAgent& v : vehicles) {
        if (v.mode != VehicleMode::ACTIVE) continue;  // DWELL/NEED_TASK do not accumulate
        if (v.action == VehicleAction::STOP ||
            v.action == VehicleAction::CREEP ||
            v.action == VehicleAction::YIELD) {
            v.wait_time += dt;
        } else {
            v.wait_time = 0.0;
        }
    }
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
