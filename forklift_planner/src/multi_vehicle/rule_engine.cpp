#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/conflict_zone_closure.h"
#include "forklift_planner/multi_vehicle/bridge_ttc_correction.h"

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
    : mp_(mp), cfg_(cfg), a1_(mp, cfg) {}

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
         << " reason=" << (reservation.create_reason.empty()
                                ? "unknown" : reservation.create_reason)
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
           a.create_reason == b.create_reason &&
           a.raw_zone_index == b.raw_zone_index &&
           a.aabb_min_x == b.aabb_min_x && a.aabb_min_y == b.aabb_min_y &&
           a.aabb_max_x == b.aabb_max_x && a.aabb_max_y == b.aabb_max_y &&
           a.aabb_valid == b.aabb_valid;
}

}  // namespace

RuleEngine::SimSnapshot RuleEngine::snapshot() const {
    return SimSnapshot{conflict_reservations_, a1_.snapshot(),
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
    conflict_reservations_ = s.reservations;
    a1_.restore(s.a1);
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

    // 搂9 鐮寸幆杞︿复鏃舵渶楂樹紭鍏堢骇:璺ㄥ眰涓€鑷粹€斺€攑airwise 涔熷繀椤昏鐮寸幆杞﹁耽,鍚﹀垯瀹冨湪璧勬簮灞?
    // 璧簡浠ょ墝銆佸嵈琚?pairwise 鎽佷綇,鐜繕鏄牬涓嶄簡銆?
    if (a.deadlock_breaker != b.deadlock_breaker) return a.deadlock_breaker ? a.id : b.id;

    // 璧勬簮鍓嶇疆绾︽潫(闈炰换鎰忎紭鍏堢骇,搂4/搂6/搂7):鑻ヤ竴杞﹁鍘荤殑鐩爣搴撲綅姝ｈ鍙︿竴杞﹀崰鐫€
    // (a.target==b.current),鍗犵敤鑰呭繀椤诲厛娓呭嚭璇ヤ綅銆佸叆搴撹€呰琛屸€斺€斿惁鍒欏叆搴撹€呮姠鍏堝紑鍒?
    // 搴撲綅鍙ｅ嵈杩涗笉鍘?浣嶈鍗?,鍗犵敤鑰呭張琚畠璁╁仠鍦ㄥ彛鍐?鐩存帴姝婚攣銆傝繖鏄?slot 璧勬簮渚濊禆,
    // 褰撲换鍔″彧鎸囧悜绌?鍗冲皢绌虹殑搴撲綅鏃舵棤鐜?鏁呯疆浜庝弗鏍煎叏搴忎箣涓婁綔 override(鑻ヤ笌鍏ㄥ簭鍐茬獊鎴愮幆
    // =浠诲姟鍒嗛厤灞傜殑寰幆鍗犱綅闂,鐢辩5姝ョ幆妫€娴嬫柇瑷€鎶撳嚭)銆?
    {
        const bool a_wants_b_slot = (a.target_slot == b.current_slot);
        const bool b_wants_a_slot = (b.target_slot == a.current_slot);
        if (a_wants_b_slot && !b_wants_a_slot) return b.id;  // b 鍗犵敤鑰?鍏堟竻鍑?
        if (b_wants_a_slot && !a_wants_b_slot) return a.id;  // a 鍗犵敤鑰?鍏堟竻鍑?
    }

    // Ordinary-road authority is the existing deterministic total order.
    // Local interaction geometry (including same-direction front/rear order)
    // is not a second priority system.
    return unifiedPriority(a, b);
}

int RuleEngine::unifiedPriority(const VehicleAgent& a,
                                const VehicleAgent& b) const {
    // Deterministic total order: loaded first, fewer completed tasks first,
    // then vehicle id as the unique final tie-breaker.
    auto key = [](const VehicleAgent& v) {
        return std::make_tuple(v.loaded ? 0 : 1, v.task_count, v.id);
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

    // 涓ゆ潯璺緞閮芥槸銆屽畬鍏ㄥ浐瀹氥€佸凡鐭ャ€嶇殑;瀹冧滑涔嬮棿鐨勫啿绐侀泦 C_ij 鏄潤鎬佸嚑浣曢噺,涓庢椂闂?
    // 閫熷害/鏈濆悜/褰撳墠浣嶇疆閮芥棤鍏?鈫?鍦ㄦ暣娈佃矾寰?[0,L]脳[0,L] 涓婁竴娆＄畻瀹氬嵆鍙?鐢?
    // conflictBlocksCanonical 鎸?path_gen 缂撳瓨)銆傚綋鍓嶄綅缃浉鍏崇殑瑁佸壀(瑁佹帀宸叉竻鍑虹殑鍧椼€?
    // 鎶婂叆鍙ｅす鍒拌溅灏捐捣鐐?鍦?findConflictZones 閲屾寜璋冪敤鏃剁殑 path_s 鏂藉姞銆?
    // (鍘嗗彶鏁欒:鏃╁厛鎸?current_speed脳prediction_horizon 瑁佸壀鎵弿鑼冨洿,浼氬湪璁╄杞﹀仠鍦?
    // 鍋滄绾?speed鈫? 鏃惰鍐茬獊鍖哄嚟绌烘秷澶?鈫?鍘熷瓙闂ㄩ绾﹁閲婃斁 鈫?璁╄杞﹁弓杩囧仠姝㈢嚎鎸よ繘鍖?
    // 鈫?涓夎溅鏃嬭浆妤旀銆傛敼鎵暣娈靛浐瀹氳矾寰勬牴娌?姝ゅ杩涗竴姝ョ紦瀛?鍑犱綍鎭掑畾鍙涓斾笉姣忔媿閲嶇畻銆?
    const double s_self_end = self.track.length();
    const double s_other_end = other.track.length();
    const double s_self_begin = 0.0;
    const double s_other_begin = 0.0;

    // 骞跨浉鍓灊(鎵叏绋嬪悗鐨勬€ц兘鎶ゆ爮):涓よ溅銆屽墿浣欒矾寰勩€嶇殑杞村榻愬寘鍥寸洅鍚勬寜銆岃溅韬瑙掔嚎
    // 鍗婂緞 + 鍐茬獊浣欓噺銆嶅鑳€;鑻ヤ袱鐩掑垎绂?鍒欎换鎰忎綅濮夸笅杞﹁韩 OBB 缁濇棤鍙兘閲嶅彔 鈫?鐩存帴杩斿洖绌?
    // 鍏嶅幓瀵瑰叓绔垮瓙鎵撲笉鐫€鐨勮繙杞﹀仛鏁存绮炬壂銆傝繖鍙渷绠楀姏,缁濅笉鏀瑰彉浠讳綍鍐茬獊鍒ゅ畾缁撴灉銆?
    {
        const double inf = std::numeric_limits<double>::infinity();
        const double infl =
            0.5 * std::hypot(mp_.vehicle_length, mp_.vehicle_width) +
            cfg_.conflict_margin;
        constexpr double kCoarse = 0.15;  // 绮楅噰鏍峰缓鍖呭洿鐩?
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
            return zones;  // 鍖呭洿鐩掑垎绂?鈫?鏃犲啿绐?
        }
    }

    for (double ss = s_self_begin; ss <= s_self_end + 1e-9; ss += kStep) {
        const double ss_clamped = std::min(ss, s_self_end);
        // Static ConflictZone uses the same bare-body geometry as the timed
        // prediction layer and the simulation hard collision guard.
        const OBB obb_s = makeBody(
            self.track.poseAtS(ss_clamped), mp_, 0.0);
        std::vector<OverlapSample> row;

        for (double so = s_other_begin; so <= s_other_end + 1e-9; so += kStep) {
            const double so_clamped = std::min(so, s_other_end);
            const OBB obb_o = makeBody(
                other.track.poseAtS(so_clamped), mp_, 0.0);

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

    // 涓烘瘡涓潡绠椼€屽悓鍚戙€嶆爣蹇?闈欐€?:鍦ㄥ潡涓偣娴嬩袱璺緞琛岃繘鏈濆悜(REVERSE 娈靛垏绾?蟺),
    // 鍚屽悜(dot>0.7)=姝ｅ瑙掑甫=鍚岃溅閬撹窡杞?鍚﹀垯浜ゅ弶/瀵瑰悜銆備緵 resolveFollowing 涓?pairwise
    // 鍏辩敤浣滅ǔ瀹氬垽鎹€斺€斾笉鍐嶇敤闅忓綋鍓嶄綅濮块棯鐑佺殑鐬椂鏈濆悜(浜ゅ弶/姹囧叆澶勪細鐬椂瀵归綈鑰岃鍒?銆?
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

const std::vector<RuleEngine::ConflictZone>& RuleEngine::conflictBlocksCanonical(
    const VehicleAgent& lo, const VehicleAgent& hi) const {
    // lo.id < hi.id(璋冪敤鏂逛繚璇?銆傛寜 path_gen 缂撳瓨:浠讳竴鏂规崲浜嗗浐瀹氳矾寰勬墠閲嶇畻銆?
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
    // 鍙栭潤鎬?C_ij(缂撳瓨),鎸?self/other 鏈濆悜鍙栫敤,鍐嶆寜褰撳墠浣嶇疆瑁佸壀鈥斺€斾骇鐗╀笌鍘嗗彶
    // 銆岄€愭媿娌垮墿浣欒矾寰勫叏绋嬬簿鎵€嶅湪鍚屼竴绂绘暎绮惧害涓嬬瓑浠?绐楀彛鍙埅浣庣銆佷笉浼氭媶鍒嗚繛閫氬潡)銆?
    const bool self_is_lo = self.id < other.id;
    const VehicleAgent& lo = self_is_lo ? self : other;
    const VehicleAgent& hi = self_is_lo ? other : self;
    const std::vector<ConflictZone>& canon = conflictBlocksCanonical(lo, hi);

    // 杞﹀熬鍙傝€?杞﹁韩鍚戝悗浼?rear_ext銆傛煇杞︺€屽凡瀹屽叏娓呭嚭鏌愬潡銆? 杞﹀熬(s-rear_ext)瓒婅繃璇ュ潡
    // 鍦ㄥ叾璺緞涓婄殑鍑哄彛 鈫?璇ュ潡瀵瑰畠涓嶅啀鏄啿绐?涓㈠純銆?
    // 娉?鍧楃殑鍏ュ彛/鍑哄彛淇濇寔銆岄潤鎬併€?涓嶅啀澶瑰埌 path_s-rear_ext)鈥斺€斿す绱т細璁╀笂鎶ョ殑 se 闅忚溅
    // 鍓嶇Щ,瀵艰嚧璁╄鏂圭殑鍋滄绾?se-front)闅忓畠涓€璧锋紓銆佹案杩滆拷涓嶄笂銆佹渶缁堣弓杩涘尯(瀹炴祴 V1鈫擵5
    // 韫挒鐨勪竴鍗婃牴鍥?銆傚仠姝㈢嚎蹇呴』鏄浐瀹氬姬闀?璁╄鏂规墠鑳界ǔ绋冲仠鍦ㄥ尯澶栥€俢ommitted/cleared 浠嶆寜
    // 闈欐€?se/sx 涓庡綋鍓?path_s 姣旇緝,璇箟涓嶅彉銆?
    const double rear_ext = mp_.body_rear_ext();
    const double self_begin = std::max(0.0, self.path_s - rear_ext);
    const double other_begin = std::max(0.0, other.path_s - rear_ext);

    std::vector<ConflictZone> out;
    out.reserve(canon.size());
    for (const ConflictZone& cz : canon) {
        ConflictZone z = cz;
        if (!self_is_lo) {  // 鏈濆悜鍙嶈浆:canonical 浠?lo 涓?self,杩欓噷 self 鏄?hi
            std::swap(z.s_self_enter, z.s_other_enter);
            std::swap(z.s_self_exit, z.s_other_exit);
        }
        // 浠讳竴鏂瑰凡瀹屽叏娓呭嚭璇ュ潡 鈫?涓嶅啀鍐茬獊,涓㈠純銆?鍏ュ彛涓嶅す绱?淇濇寔闈欐€?
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
                self.track.poseAtS(self_s), mp_, 0.0);
            for (double so = zone.s_other_enter;
                 so <= zone.s_other_exit + 1e-9; so += kDisplayStep) {
                const double other_s = std::min(so, zone.s_other_exit);
                const OBB other_body = makeBody(
                    other.track.poseAtS(other_s), mp_, 0.0);
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
    const std::vector<ConflictMarker::TimedOverlap>& timed_overlaps,
    PairInteractionType interaction_type, double last_conflict_t) {
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

        if (interaction_type == PairInteractionType::OPPOSING &&
            z.aabb_valid) {
            x_min = z.aabb_min_x;
            y_min = z.aabb_min_y;
            x_max = z.aabb_max_x;
            y_max = z.aabb_max_y;
        } else if (!timed_overlaps.empty()) {
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
        if (z.raw_index >= 0) {
            const auto active_zones = findConflictZones(self, other);
            for (size_t active = 0; active < active_zones.size(); ++active) {
                if (active_zones[active].raw_index == z.raw_index) {
                    marker.active_zone_index = static_cast<int>(active);
                    break;
                }
            }
        }
        marker.follower_id = follower_id;
        marker.leader_id = leader_id;
        marker.holder_id = holder_id;
        marker.waiter_id = waiter_id;
        marker.following_gap = following_gap;
        marker.following_action = following_action;
        marker.interaction_type = interaction_type;
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
        marker.last_t = last_conflict_t >= 0.0
            ? last_conflict_t : first_conflict_t;
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
    a1_.refreshDepartureClusters(
        vehicles, [this](const VehicleAgent& first,
                         const VehicleAgent& second) {
            const VehicleAgent& lo = first.id < second.id ? first : second;
            const VehicleAgent& hi = first.id < second.id ? second : first;
            const auto& cached = conflictBlocksCanonical(lo, hi);
            return std::vector<ConflictZone>(cached.begin(), cached.end());
        });
    return;
}

int RuleEngine::departureClusterOwnerForPair(const VehicleAgent& a,
                                             const VehicleAgent& b) const {
    return a1_.departureAuthorityForPair(a, b).owner_id;
}

RuleEngine::A1LaunchAdmission RuleEngine::checkA1LaunchAdmission(
    const VehicleAgent& service_owner,
    const VehicleAgent& launch_candidate) const {
    return a1_.checkLaunchAdmission(
        service_owner, launch_candidate,
        [this](const VehicleAgent& first, const VehicleAgent& second) {
            return computeConflictZonesFull(first, second);
        });
}

RuleEngine::SlotDepartureAdmission RuleEngine::checkSlotDepartureAdmission(
    const VehicleAgent* service_owner,
    const VehicleAgent& launch_candidate,
    const std::vector<VehicleAgent>& vehicles,
    double prediction_horizon) const {
    SlotDepartureAdmission result;
    if (!launch_candidate.active() || launch_candidate.track.empty() ||
        launch_candidate.mission_phase != MissionPhase::TO_A1) {
        return result;
    }

    if (service_owner != nullptr &&
        service_owner->id != launch_candidate.id) {
        result.a1 = checkA1LaunchAdmission(*service_owner,
                                           launch_candidate);
        if (result.a1.departure_resource_conflict) {
            result.clear = false;
            result.a1_departure_conflict = true;
            result.blocker_id = service_owner->id;
        }
    }

    const double horizon = std::max(cfg_.prediction_step,
                                    prediction_horizon);
    const auto candidate_prediction = predictTrajectory(
        launch_candidate, mp_, cfg_, VehicleAction::NOMINAL, horizon);
    if (candidate_prediction.empty()) return result;

    auto sampleAt = [](const auto& prediction, double time)
        -> const PredictedKinematicSample& {
        auto it = std::lower_bound(
            prediction.begin(), prediction.end(), time,
            [](const PredictedKinematicSample& sample, double value) {
                return sample.t < value;
            });
        return it == prediction.end() ? prediction.back() : *it;
    };
    constexpr double kPi = 3.14159265358979323846;
    auto motionHeading = [&](const VehicleAgent& vehicle, double s) {
        double heading = vehicle.track.poseAtS(s).theta;
        if (vehicle.track.typeAtS(s) == WpType::REVERSE) heading += kPi;
        return heading;
    };

    for (const VehicleAgent& occupant : vehicles) {
        if (occupant.id == launch_candidate.id || !occupant.active() ||
            occupant.track.empty()) {
            continue;
        }
        const auto occupant_prediction = predictTrajectory(
            occupant, mp_, cfg_, VehicleAction::NOMINAL, horizon);
        const PairInteractionResult physical =
            detectPairInteractionFromPredictions(
                launch_candidate, occupant, {}, candidate_prediction,
                occupant_prediction);
        if (!physical.event.valid) continue;

        const PredictedKinematicSample& candidate_sample = sampleAt(
            candidate_prediction, physical.event.first_overlap_t);
        if (candidate_sample.s >
            launch_candidate.slot_departure_clear_s + 1e-9) {
            continue;
        }
        if (result.ordinary_road_conflict &&
            physical.event.first_overlap_t >= result.first_conflict_t - 1e-9) {
            continue;
        }

        const PredictedKinematicSample& occupant_sample = sampleAt(
            occupant_prediction, physical.event.first_overlap_t);
        const double direction_dot = std::cos(
            motionHeading(launch_candidate, candidate_sample.s) -
            motionHeading(occupant, occupant_sample.s));
        result.interaction_type = direction_dot < -0.5
            ? PairInteractionType::OPPOSING
            : direction_dot > 0.7
                ? PairInteractionType::SAME_DIRECTION
                : PairInteractionType::CROSSING;
        result.clear = false;
        result.ordinary_road_conflict = true;
        result.blocker_id = occupant.id;
        result.first_conflict_t = physical.event.first_overlap_t;
        result.candidate_conflict_s = candidate_sample.s;
    }
    return result;
}

void RuleEngine::enforceDepartureClusterCommitments(
    std::vector<VehicleAgent>& vehicles, double dt) {
    const auto requests = a1_.enforceDepartureClusters(vehicles, dt);
    for (const A1ActionRequest& request : requests) {
        for (VehicleAgent& vehicle : vehicles) {
            if (vehicle.id == request.vehicle_id) {
                applyActionRequest(vehicle, request.action, request.reason,
                                   request.blocker_id);
                break;
            }
        }
    }
    return;
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
    PairInteractionResult result = detectPairInteractionFromPredictions(
        a, b, {}, prediction_a, prediction_b);
    // Static path geometry remains diagnostic only for this public query; it
    // is no longer a gate or identity source for a crossing TimedConflict.
    result.potential_zones = zones;
    return result;
}

void RuleEngine::resolvePairwiseConflicts(std::vector<VehicleAgent>& vehicles,
                                          double dt,
                                          double prediction_horizon,
                                          bool reuse_ordinary_coordination) {
    pairwise_managed_pairs_.clear();
    ordinary_dynamic_pairs_.clear();
    following_pairs_.clear();
    // Predict each vehicle once. Pair detection below is pure and consumes
    // these shared samples without changing coordination state.
    const double horizon =
        std::max(cfg_.prediction_step, prediction_horizon);
    const double prediction_step = std::max(0.02, cfg_.prediction_step);
    std::vector<std::vector<PredictedKinematicSample>> predictions(
        vehicles.size());
    for (size_t i = 0; i < vehicles.size(); ++i) {
        if (vehicles[i].active() && !vehicles[i].track.empty()) {
            const VehicleAction prediction_action =
                vehicles[i].ttc_stop_hold_remaining > 1e-9
                    ? VehicleAction::STOP : VehicleAction::NOMINAL;
            predictions[i] =
                predictTrajectory(vehicles[i], mp_, cfg_,
                                  prediction_action, horizon);
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
    const double a1_control_stop_margin = cfg_.a1_control_stop_margin;
    auto insideInterval = [](const VehicleAgent& v, double enter_s,
                             double exit_s) {
        return v.path_s > enter_s + 1e-9 &&
               v.path_s <= exit_s + 1e-9;
    };
    auto brakeBefore = [&](VehicleAgent& v, double conflict_enter_s,
                           int other_id) {
        if (v.deadlock_breaker) return;
        const double physical_entry_s = conflict_enter_s;
        const double control_stop_s = std::max(
            0.0, physical_entry_s - a1_control_stop_margin);
        const double distance = control_stop_s - v.path_s;
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
    // Every active pair, including pairs in a 3+ vehicle scene, is handled by
    // the same rolling motion coordinator. Pair outputs are merged later by
    // applyActionRequest() using the existing restrictive-action ordering.
    const bool dynamic_speed_enabled = vehicles.size() >= 2;
    auto logNominalRecovery = [&](const std::pair<int, int>& key) {
        if (!dynamic_speed_enabled || reuse_ordinary_coordination) return;
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

    // Stage 3.1 boundary: only A1 service transactions may retain cross-period
    // pair ownership. Remove any ordinary-road reservation restored from an
    // older snapshot before it can skip rolling motion coordination.
    for (auto it = conflict_reservations_.begin();
         it != conflict_reservations_.end();) {
        VehicleAgent* lo = agentById(it->first.first);
        VehicleAgent* hi = agentById(it->first.second);
        const ConflictReservation& r = it->second;
        if (r.create_reason != "a1_related" ||
            lo == nullptr || hi == nullptr || !lo->active() || !hi->active() ||
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
        const auto compute_full =
            [this](const VehicleAgent& first, const VehicleAgent& second) {
                return computeConflictZonesFull(first, second);
            };
        const auto canonical =
            [this](const VehicleAgent& first, const VehicleAgent& second) {
                const VehicleAgent& lo = first.id < second.id ? first : second;
                const VehicleAgent& hi = first.id < second.id ? second : first;
                const auto& cached = conflictBlocksCanonical(lo, hi);
                return std::vector<ConflictZone>(cached.begin(), cached.end());
            };
        return a1_.futureAuthorityForPair(
            a, b, compute_full, canonical).owner_id;
    };

    auto eventZone = [&](const PairInteractionResult& interaction,
                         const std::vector<PredictedKinematicSample>& pa,
                         const std::vector<PredictedKinematicSample>& pb) {
        ConflictZone zone;
        auto sampleAt = [](const auto& prediction, double time)
            -> const PredictedKinematicSample& {
            auto it = std::lower_bound(
                prediction.begin(), prediction.end(), time,
                [](const PredictedKinematicSample& sample, double value) {
                    return sample.t < value;
                });
            return it == prediction.end() ? prediction.back() : *it;
        };
        const auto& first_a = sampleAt(pa, interaction.event.first_overlap_t);
        const auto& last_a = sampleAt(pa, interaction.event.last_t);
        const auto& first_b = sampleAt(pb, interaction.event.first_overlap_t);
        const auto& last_b = sampleAt(pb, interaction.event.last_t);
        zone.s_self_enter = first_a.s;
        zone.s_self_exit = last_a.s;
        zone.s_other_enter = first_b.s;
        zone.s_other_exit = last_b.s;
        for (const TimedOverlapGeometry& overlap :
             interaction.event.timed_overlaps) {
            for (const InteractionPoint& point : overlap.polygon) {
                if (!zone.aabb_valid) {
                    zone.aabb_min_x = zone.aabb_max_x = point.x;
                    zone.aabb_min_y = zone.aabb_max_y = point.y;
                    zone.aabb_valid = true;
                } else {
                    zone.aabb_min_x = std::min(zone.aabb_min_x, point.x);
                    zone.aabb_min_y = std::min(zone.aabb_min_y, point.y);
                    zone.aabb_max_x = std::max(zone.aabb_max_x, point.x);
                    zone.aabb_max_y = std::max(zone.aabb_max_y, point.y);
                }
            }
        }
        if (zone.aabb_valid) {
            zone.x = 0.5 * (zone.aabb_min_x + zone.aabb_max_x);
            zone.y = 0.5 * (zone.aabb_min_y + zone.aabb_max_y);
        }
        return zone;
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
            std::vector<ConflictZone> zones;
            // Crossing truth is generated directly from synchronized OBBs.
            // Static path geometry is intentionally absent from this call.
            const PairInteractionResult direct_interaction =
                detectPairInteractionFromPredictions(
                    a, b, {}, predictions[i], predictions[j]);

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
                             << " selection=SKIPPED"
                             << " reason=existing_reservation"
                             << " reservation_reason="
                             << (reservation_it->second.create_reason.empty()
                                     ? "unknown"
                                     : reservation_it->second.create_reason);
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
                    ++dynamic_speed_metrics_.reservation_updates;
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
                            direct_interaction.event.timed_overlaps);
                    pairwise_managed_pairs_.insert(key);
                    const bool owner_inside =
                        insideInterval(owner, owner_enter, owner_exit);
                    const bool waiter_inside =
                        insideInterval(waiter, waiter_enter, waiter_exit);
                    if (waiter_inside && !owner_inside &&
                        owner.path_s <= owner_enter + 1e-9) {
                        // Current physical occupancy overrides an old forecast.
                        r.owner_id = waiter.id;
                        ++dynamic_speed_metrics_.reservation_updates;
                        logConflictReservation(coord_log_sink_, key, "update", r);
                        brakeBefore(owner, owner_enter, waiter.id);
                        if (owner.reason ==
                            "time_brake_V" + std::to_string(waiter.id)) {
                            if (a1_.shouldLogDecision(owner, waiter.id)) {
                                a1_.logDecision(owner, &waiter, waiter.id);
                            }
                        }
                    } else {
                        brakeBefore(waiter, waiter_enter, owner.id);
                        if (waiter.reason ==
                            "time_brake_V" + std::to_string(owner.id)) {
                            if (a1_.shouldLogDecision(waiter, owner.id)) {
                                a1_.logDecision(waiter, &owner, owner.id);
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

            int departure_cluster_owner =
                departureClusterOwnerForPair(a, b);
            int future_owner = futureA1OwnerForPair(a, b);
            // A1 ownership is pair/resource scoped, not owner-identity scoped.
            // A staged handoff or a vehicle's departure flag alone does not
            // remove its current-road interactions from rolling coordination.
            const bool a1_related =
                departure_cluster_owner >= 0 || future_owner >= 0;
            const bool ordinary = !a1_related;

            PairInteractionResult interaction;
            if (a1_related) {
                // A1 is the only remaining consumer of the legacy fixed-zone
                // association and reservation lifecycle.
                zones = findConflictZones(a, b);
                interaction = detectPairInteractionFromPredictions(
                    a, b, zones, predictions[i], predictions[j]);
            } else {
                // Ordinary ACTIVE-ACTIVE pairs have one physical authority:
                // synchronized OBB overlap over the rolling horizon.
                interaction = direct_interaction;
            }

            int ordinary_priority_id = -1;
            PriorityPhysicalTtcEvaluation priority_physical;
            if (ordinary && !reuse_ordinary_coordination) {
                ordinary_priority_id = priorityWinner(a, b);
                const bool a_is_priority = ordinary_priority_id == a.id;
                const VehicleAgent& priority_vehicle = a_is_priority ? a : b;
                const VehicleAgent& other_vehicle = a_is_priority ? b : a;
                const auto& priority_prediction =
                    a_is_priority ? predictions[i] : predictions[j];
                if (priority_vehicle.ttc_stop_hold_remaining <= 1e-9) {
                    priority_physical = evaluatePriorityPhysicalTtc(
                        priority_vehicle, other_vehicle, priority_prediction,
                        mp_, cfg_);
                }
            }

            const TimedConflictEvent& event = interaction.event;
            if (!event.valid) {
                bool priority_physical_stop = false;
                if (ordinary && priority_physical.valid) {
                    const TtcStopBoundary boundary = evaluateTtcStopBoundary(
                        priority_physical.safety_ttc,
                        VehicleAction::NOMINAL, cfg_);
                    if (boundary.stop_required) {
                        VehicleAgent& priority_vehicle =
                            ordinary_priority_id == a.id ? a : b;
                        const VehicleAgent& other_vehicle =
                            ordinary_priority_id == a.id ? b : a;
                        priority_vehicle.ttc_stop_hold_remaining = std::max(
                            priority_vehicle.ttc_stop_hold_remaining,
                            cfg_.rolling_refresh_period);
                        applyActionRequest(
                            priority_vehicle, VehicleAction::STOP,
                            "dynamic_speed_STOP_V" +
                                std::to_string(ordinary_priority_id),
                            other_vehicle.id);
                        priority_physical_stop = true;
                        last_rolling_dynamic_decision_.valid = true;
                        last_rolling_dynamic_decision_.baseline_evaluated = true;
                        last_rolling_dynamic_decision_.emergency_stop = true;
                        last_rolling_dynamic_decision_.selected_action_a =
                            ordinary_priority_id == a.id
                                ? VehicleAction::STOP : VehicleAction::NOMINAL;
                        last_rolling_dynamic_decision_.selected_action_b =
                            ordinary_priority_id == b.id
                                ? VehicleAction::STOP : VehicleAction::NOMINAL;
                        last_rolling_dynamic_decision_.targets.push_back(
                            RollingDynamicDecision::Target{
                                priority_vehicle.id,
                                priority_vehicle.path_gen,
                                VehicleAction::STOP,
                                priority_vehicle.action,
                                other_vehicle.id,
                                priority_vehicle.reason});
                        last_rolling_dynamic_decision_.vehicle_ttc_diagnostics.
                            push_back(
                                RollingDynamicDecision::VehicleTtcDiagnostic{
                                    priority_vehicle.id,
                                    priority_vehicle.path_gen,
                                    priority_physical.safety_ttc,
                                    priority_physical.bridge_related
                                        ? "bridge_physical_safety_stop"
                                        : "physical_safety_stop"});
                        if (coord_log_sink_) {
                            std::ostringstream line;
                            line << "[DYN-PHYSICAL] pair=V" << key.first
                                 << "-V" << key.second
                                 << " priority=V" << priority_vehicle.id
                                 << " other_current=V" << other_vehicle.id
                                 << " collision_t="
                                 << priority_physical.collision_t
                                 << " collision_s="
                                 << priority_physical.collision_s
                                 << " safety_ttc="
                                 << priority_physical.safety_ttc
                                 << " boundary_s="
                                 << priority_physical.safety_boundary_s
                                 << " bridge="
                                 << (priority_physical.bridge_related
                                         ? "true" : "false")
                                 << " selected=STOP/NOMINAL"
                                 << " reason=priority_physical_safety_stop";
                            coord_log_sink_(line.str());
                        }
                    }
                }
                if (dynamic_speed_enabled) {
                    last_rolling_dynamic_decision_.baseline_evaluated = true;
                }
                if (!priority_physical_stop) logNominalRecovery(key);
                continue;
            }

            // The bridge layer is a stateless correction of an already-real
            // ordinary synchronized-OBB conflict. A clear baseline never
            // reaches it, and A1 keeps its existing resource path unchanged.
            PairBridgeTtcCorrection bridge_correction;
            if (ordinary) {
                bridge_correction = evaluateBridgeTtcCorrection(
                    a, b, predictions[i], predictions[j], interaction,
                    mp_, cfg_);
                interaction.type =
                    bridge_correction.a.bridge_related ||
                            bridge_correction.b.bridge_related
                        ? PairInteractionType::OPPOSING
                        : PairInteractionType::CROSSING;
            }
            auto annotateBridgeMarker = [&]() {
                if (conflicts_.empty()) return;
                ConflictMarker& marker = conflicts_.back();
                if (marker.vehicle_a != a.id || marker.vehicle_b != b.id) {
                    return;
                }
                marker.bridge_a_related =
                    bridge_correction.a.bridge_related;
                marker.bridge_b_related =
                    bridge_correction.b.bridge_related;
                const RoughWp boundary_a = a.track.poseAtS(
                    bridge_correction.a.near_boundary_s);
                const RoughWp boundary_b = b.track.poseAtS(
                    bridge_correction.b.near_boundary_s);
                marker.bridge_boundary_a_x = boundary_a.x;
                marker.bridge_boundary_a_y = boundary_a.y;
                marker.bridge_boundary_b_x = boundary_b.x;
                marker.bridge_boundary_b_y = boundary_b.y;
                marker.bridge_corrected_ttc_a =
                    bridge_correction.a.corrected_ttc;
                marker.bridge_corrected_ttc_b =
                    bridge_correction.b.corrected_ttc;
            };
            auto annotateTimedCollisionStartMarker = [&]() {
                if (conflicts_.empty()) return;
                ConflictMarker& marker = conflicts_.back();
                if (marker.vehicle_a != a.id || marker.vehicle_b != b.id ||
                    marker.kind !=
                        ConflictMarkerKind::CROSSING_OR_OPPOSING) {
                    return;
                }
                const RoughWp collision_a = a.track.poseAtS(
                    event.collision_s_a);
                const RoughWp collision_b = b.track.poseAtS(
                    event.collision_s_b);
                marker.timed_collision_start_valid = true;
                marker.collision_s_a = event.collision_s_a;
                marker.collision_s_b = event.collision_s_b;
                marker.collision_a_x = collision_a.x;
                marker.collision_a_y = collision_a.y;
                marker.collision_b_x = collision_b.x;
                marker.collision_b_y = collision_b.y;
            };

            pairwise_managed_pairs_.insert(key);
            if (ordinary) ordinary_dynamic_pairs_.insert(key);

            ConflictZone zone;
            if (a1_related) {
                if (event.associated_zone_index < 0 ||
                    static_cast<size_t>(event.associated_zone_index) >=
                        zones.size()) {
                    continue;
                }
                zone = zones[static_cast<size_t>(
                    event.associated_zone_index)];
            } else {
                zone = eventZone(interaction, predictions[i], predictions[j]);
            }
            const bool a_inside =
                insideInterval(a, zone.s_self_enter, zone.s_self_exit);
            const bool b_inside =
                insideInterval(b, zone.s_other_enter, zone.s_other_exit);
            const bool a_terminal = terminalDocking(a);
            const bool b_terminal = terminalDocking(b);

            // The rolling-period target was selected from the true state at
            // frame 0.  Future sandbox states may still run reservation/A1 and
            // safety rules, but must not turn the same period's FAR into MID,
            // re-run priority/candidates, or create a new ordinary reservation.
            if (ordinary && reuse_ordinary_coordination) {
                recordConflictZones(
                    a, b, std::vector<ConflictZone>{zone},
                    ConflictMarkerKind::CROSSING_OR_OPPOSING,
                    event.first_overlap_t, -1, -1, 0.0,
                    VehicleAction::NOMINAL, -1, -1,
                    decimateTimedOverlaps(event.timed_overlaps),
                    interaction.type, event.last_t);
                annotateTimedCollisionStartMarker();
                annotateBridgeMarker();
                continue;
            }

            if (dynamic_speed_enabled) {
                ++dynamic_speed_metrics_.baseline_conflicts;
                if (ordinary) {
                    ++dynamic_speed_metrics_.bridge_checked_pairs;
                    dynamic_speed_metrics_.bridge_nearest_evaluations +=
                        bridge_correction.a.nearest_search_evaluations +
                        bridge_correction.b.nearest_search_evaluations;
                    dynamic_speed_metrics_.bridge_backtrack_samples +=
                        bridge_correction.a.backtrack_samples +
                        bridge_correction.b.backtrack_samples;
                    dynamic_speed_metrics_.bridge_max_backtrack_samples =
                        std::max(
                            dynamic_speed_metrics_.bridge_max_backtrack_samples,
                            static_cast<unsigned long long>(std::max(
                                bridge_correction.a.backtrack_samples,
                                bridge_correction.b.backtrack_samples)));
                    if (bridge_correction.a.bridge_related) {
                        ++dynamic_speed_metrics_.bridge_related_a;
                    }
                    if (bridge_correction.b.bridge_related) {
                        ++dynamic_speed_metrics_.bridge_related_b;
                    }
                    if (bridge_correction.a.corrected_ttc + 1e-9 <
                            event.ttc_a ||
                        bridge_correction.b.corrected_ttc + 1e-9 <
                            event.ttc_b) {
                        ++dynamic_speed_metrics_.bridge_corrected_pairs;
                    }
                    if (interaction.type == PairInteractionType::OPPOSING) {
                        ++dynamic_speed_metrics_.opposing_conflicts;
                    } else {
                        ++dynamic_speed_metrics_.crossing_conflicts;
                    }
                }
                int preferred_winner = -1;
                if (ordinary) {
                    preferred_winner = ordinary_priority_id;
                }

                PairSpeedCoordinationResult speed_result;
                if (ordinary) {
                    const bool a_is_priority = preferred_winner == a.id;
                    const double effective_ttc_a = std::min(
                        event.ttc_a, bridge_correction.a.corrected_ttc);
                    const double effective_ttc_b = std::min(
                        event.ttc_b, bridge_correction.b.corrected_ttc);
                    PairInteractionResult effective_interaction = interaction;
                    effective_interaction.event.danger_s_a =
                        bridge_correction.a.near_boundary_s;
                    effective_interaction.event.danger_s_b =
                        bridge_correction.b.near_boundary_s;
                    effective_interaction.event.ttc_a =
                        effective_ttc_a;
                    effective_interaction.event.ttc_b =
                        effective_ttc_b;
                    speed_result = evaluatePairSpeedCoordination(
                        a, b, effective_interaction, priority_physical,
                        cfg_, preferred_winner);
                    preferred_winner = speed_result.selected_winner_id;
                    const DynamicInterventionBand intervention_band =
                        speed_result.yielding_band;
                    last_rolling_dynamic_decision_.valid = true;
                    last_rolling_dynamic_decision_.baseline_evaluated = true;
                    last_rolling_dynamic_decision_.band = intervention_band;
                    last_rolling_dynamic_decision_.legacy_fallback = false;
                    last_rolling_dynamic_decision_.emergency_stop =
                        speed_result.emergency_stop;
                    last_rolling_dynamic_decision_.selected_action_a =
                        speed_result.selected_action_a;
                    last_rolling_dynamic_decision_.selected_action_b =
                        speed_result.selected_action_b;
                    last_rolling_dynamic_decision_.baseline_first_overlap_t =
                        event.first_overlap_t;
                    auto recordVehicleTtc = [&](
                            const VehicleAgent& vehicle,
                            const std::optional<double>& ttc,
                            const std::string& reason) {
                        auto& diagnostics = last_rolling_dynamic_decision_.
                            vehicle_ttc_diagnostics;
                        auto existing = std::find_if(
                            diagnostics.begin(), diagnostics.end(),
                            [&](const RollingDynamicDecision::
                                    VehicleTtcDiagnostic& diagnostic) {
                                return diagnostic.vehicle_id == vehicle.id &&
                                       diagnostic.path_gen == vehicle.path_gen;
                            });
                        const RollingDynamicDecision::VehicleTtcDiagnostic
                            incoming{vehicle.id, vehicle.path_gen, ttc, reason};
                        if (existing == diagnostics.end()) {
                            diagnostics.push_back(incoming);
                        } else if (ttc &&
                                   (!existing->ttc ||
                                    *ttc < *existing->ttc)) {
                            *existing = incoming;
                        }
                    };
                    const std::optional<double> priority_ttc =
                        speed_result.priority_physical_ttc;
                    const std::string priority_reason =
                        speed_result.priority_safety_stop
                            ? "safety_stop"
                            : priority_ttc ? "physical_safe" : "clear";
                    recordVehicleTtc(
                        a, a_is_priority
                               ? priority_ttc
                               : speed_result.yielding_effective_ttc,
                        a_is_priority ? priority_reason
                                      : speed_result.reason);
                    recordVehicleTtc(
                        b, a_is_priority
                               ? speed_result.yielding_effective_ttc
                               : priority_ttc,
                        a_is_priority ? speed_result.reason
                                      : priority_reason);
                    if (intervention_band == DynamicInterventionBand::FAR) {
                        ++dynamic_speed_metrics_.far_decisions;
                    } else if (intervention_band ==
                               DynamicInterventionBand::MID) {
                        ++dynamic_speed_metrics_.mid_decisions;
                    } else {
                        ++dynamic_speed_metrics_.near_decisions;
                    }
                    if (speed_result.emergency_stop) {
                        ++dynamic_speed_metrics_.emergency_stop_decisions;
                    }

                    const VehicleAction loser_action =
                        preferred_winner == a.id
                            ? speed_result.selected_action_b
                            : preferred_winner == b.id
                                ? speed_result.selected_action_a
                                : VehicleAction::STOP;
                    if (loser_action == VehicleAction::YIELD) {
                        ++dynamic_speed_metrics_.yield_evaluations;
                    } else if (loser_action == VehicleAction::CREEP) {
                        ++dynamic_speed_metrics_.creep_evaluations;
                    }

                    const VehicleAction previous_a = a.action;
                    const VehicleAction previous_b = b.action;
                    if (speed_result.priority_safety_stop) {
                        VehicleAgent& priority_vehicle =
                            a_is_priority ? a : b;
                        if (priority_vehicle.ttc_stop_hold_remaining <= 1e-9) {
                            priority_vehicle.ttc_stop_hold_remaining =
                                cfg_.rolling_refresh_period;
                        }
                    }
                    if (speed_result.yielding_safety_stop) {
                        VehicleAgent& yielding_vehicle =
                            a_is_priority ? b : a;
                        if (yielding_vehicle.ttc_stop_hold_remaining <= 1e-9) {
                            yielding_vehicle.ttc_stop_hold_remaining =
                                cfg_.rolling_refresh_period;
                        }
                    }
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
                    for (const auto& item :
                         {std::make_pair(&a, previous_a),
                          std::make_pair(&b, previous_b)}) {
                        const VehicleAgent* vehicle = item.first;
                        if (vehicle->requested_action ==
                                VehicleAction::NOMINAL ||
                            vehicle->reason.rfind("dynamic_speed_", 0) != 0) {
                            continue;
                        }
                        const RollingDynamicDecision::Target aggregate{
                            vehicle->id, vehicle->path_gen,
                            vehicle->requested_action, item.second,
                            vehicle->blocker_id, vehicle->reason};
                        auto existing_target = std::find_if(
                            last_rolling_dynamic_decision_.targets.begin(),
                            last_rolling_dynamic_decision_.targets.end(),
                            [&](const RollingDynamicDecision::Target& target) {
                                return target.vehicle_id == vehicle->id &&
                                       target.path_gen == vehicle->path_gen;
                            });
                        if (existing_target ==
                            last_rolling_dynamic_decision_.targets.end()) {
                            last_rolling_dynamic_decision_.targets.push_back(
                                aggregate);
                        } else {
                            *existing_target = aggregate;
                        }
                    }

                    if (coord_log_sink_) {
                        const VehicleAction priority_action = a_is_priority
                            ? speed_result.selected_action_a
                            : speed_result.selected_action_b;
                        const VehicleAction yielding_action = a_is_priority
                            ? speed_result.selected_action_b
                            : speed_result.selected_action_a;
                        std::ostringstream ttc_line;
                        ttc_line << std::fixed << std::setprecision(3)
                                 << "[DYN-TTC] pair=V" << key.first << "-V"
                                 << key.second
                                 << " first_overlap_t="
                                 << event.first_overlap_t
                                 << " collision_s_a=" << event.collision_s_a
                                 << " collision_s_b=" << event.collision_s_b
                                 << " danger_s_a="
                                 << bridge_correction.a.near_boundary_s
                                 << " danger_s_b="
                                 << bridge_correction.b.near_boundary_s
                                 << " original_ttc_a=" << event.ttc_a
                                 << " original_ttc_b=" << event.ttc_b
                                 << " effective_ttc_a="
                                 << effective_ttc_a
                                 << " effective_ttc_b="
                                 << effective_ttc_b
                                 << " priority=V" << preferred_winner
                                 << " yield=V"
                                 << (a_is_priority ? b.id : a.id)
                                 << " priority_physical_ttc=";
                        if (speed_result.priority_physical_ttc) {
                            ttc_line << *speed_result.priority_physical_ttc;
                        } else {
                            ttc_line << "CLEAR";
                        }
                        ttc_line << " priority_physical_bridge="
                                 << (speed_result.priority_physical_bridge
                                         ? "true" : "false")
                                 << " yield_effective_ttc="
                                 << *speed_result.yielding_effective_ttc;
                        coord_log_sink_(ttc_line.str());

                        std::ostringstream line;
                        line << std::fixed << std::setprecision(3)
                             << "[DYN-SPEED] pair=V" << key.first << "-V"
                             << key.second
                             << " first_overlap_t=" << event.first_overlap_t
                             << " interaction=GENERIC_TIMED_CONFLICT"
                             << " priority_vehicle=V" << preferred_winner
                             << " yielding_vehicle=V"
                             << (preferred_winner == a.id ? b.id : a.id)
                             << " s_a=" << a.path_s
                             << " s_b=" << b.path_s
                             << " wait_a=" << a.wait_time
                             << " wait_b=" << b.wait_time
                             << " speed_a=" << a.current_speed
                             << " speed_b=" << b.current_speed
                             << " event_exit_a=" << zone.s_self_exit
                             << " event_exit_b=" << zone.s_other_exit
                             << " band="
                             << dynamicInterventionBandName(intervention_band)
                             << " priority_physical_ttc=";
                        if (speed_result.priority_physical_ttc) {
                            line << *speed_result.priority_physical_ttc;
                        } else {
                            line << "CLEAR";
                        }
                        line << " priority_action="
                             << actionName(priority_action)
                             << " priority_stop_threshold=";
                        if (speed_result.priority_stop_threshold) {
                            line << *speed_result.priority_stop_threshold;
                        } else {
                            line << "N/A";
                        }
                        line << " priority_safety_stop="
                             << (speed_result.priority_safety_stop
                                     ? "true" : "false")
                             << " priority_physical_bridge="
                             << (speed_result.priority_physical_bridge
                                     ? "true" : "false")
                             << " yield_effective_ttc="
                             << *speed_result.yielding_effective_ttc
                             << " yield_action=" << actionName(yielding_action)
                             << " yield_stop_threshold=";
                        if (speed_result.yielding_stop_threshold) {
                            line << *speed_result.yielding_stop_threshold;
                        } else {
                            line << "N/A";
                        }
                        line << " yield_safety_stop="
                             << (speed_result.yielding_safety_stop
                                     ? "true" : "false")
                             << " selected="
                             << actionName(speed_result.selected_action_a)
                             << "/"
                             << actionName(speed_result.selected_action_b)
                             << " braking_stop="
                             << (speed_result.emergency_stop ? "true" : "false")
                             << " residual_evaluation=DISABLED"
                             << " reason=" << speed_result.reason
                             << " reservation=not_created";
                        coord_log_sink_(line.str());

                        std::ostringstream bridge_line;
                        bridge_line << std::fixed << std::setprecision(3)
                            << "[BRIDGE-TTC] pair=V" << key.first << "-V"
                            << key.second
                            << " first_overlap_t=" << event.first_overlap_t
                            << " V" << a.id
                            << "_original_danger_s=" << event.collision_s_a
                            << " V" << a.id
                            << "_corrected_boundary_s="
                            << bridge_correction.a.near_boundary_s
                            << " V" << a.id << "_opposing_boundary_s="
                            << bridge_correction.a.opposing_boundary_s
                            << " V" << a.id << "_geometric_boundary_s="
                            << bridge_correction.a.geometric_boundary_s
                            << " V" << a.id << "_original_ttc="
                            << bridge_correction.a.original_ttc
                            << " V" << a.id << "_corrected_ttc="
                            << bridge_correction.a.corrected_ttc
                            << " V" << b.id
                            << "_original_danger_s=" << event.collision_s_b
                            << " V" << b.id
                            << "_corrected_boundary_s="
                            << bridge_correction.b.near_boundary_s
                            << " V" << b.id << "_opposing_boundary_s="
                            << bridge_correction.b.opposing_boundary_s
                            << " V" << b.id << "_geometric_boundary_s="
                            << bridge_correction.b.geometric_boundary_s
                            << " V" << b.id << "_original_ttc="
                            << bridge_correction.b.original_ttc
                            << " V" << b.id << "_corrected_ttc="
                            << bridge_correction.b.corrected_ttc
                            << " bridge_a="
                            << (bridge_correction.a.bridge_related
                                    ? "true" : "false")
                            << " bridge_b="
                            << (bridge_correction.b.bridge_related
                                    ? "true" : "false")
                            << " match_s_a="
                            << bridge_correction.a.matched_other_s
                            << " match_s_b="
                            << bridge_correction.b.matched_other_s
                            << " direction_dot_a="
                            << bridge_correction.a.collision_direction_dot
                            << " direction_dot_b="
                            << bridge_correction.b.collision_direction_dot
                            << " match_distance_a="
                            << bridge_correction.a.collision_match_distance
                            << " match_distance_b="
                            << bridge_correction.b.collision_match_distance
                            << " collision_type_a="
                            << (bridge_correction.a.collision_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " collision_type_b="
                            << (bridge_correction.b.collision_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " boundary_s_a="
                            << bridge_correction.a.near_boundary_s
                            << " boundary_s_b="
                            << bridge_correction.b.near_boundary_s
                            << " boundary_type_a="
                            << (bridge_correction.a.boundary_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " boundary_type_b="
                            << (bridge_correction.b.boundary_type ==
                                        WpType::REVERSE ? "R" : "F")
                            << " priority_vehicle=V" << preferred_winner
                            << " yielding_vehicle=V"
                            << (preferred_winner == a.id ? b.id : a.id)
                            << " yielding_ttc="
                            << *speed_result.yielding_effective_ttc
                            << " band="
                            << dynamicInterventionBandName(intervention_band)
                            << " selected="
                            << actionName(speed_result.selected_action_a)
                            << "/"
                            << actionName(speed_result.selected_action_b)
                            << " backtrack_samples="
                            << bridge_correction.a.backtrack_samples << "/"
                            << bridge_correction.b.backtrack_samples
                            << " self_traversal_changes="
                            << bridge_correction.a.self_traversal_changes
                            << "/"
                            << bridge_correction.b.self_traversal_changes
                            << " nearest_other_traversal_changes="
                            << bridge_correction.a.
                                   nearest_other_traversal_changes
                            << "/"
                            << bridge_correction.b.
                                   nearest_other_traversal_changes
                            << " backtrack_end_reason="
                            << bridgeBacktrackEndReasonName(
                                   bridge_correction.a.backtrack_end_reason)
                            << "/"
                            << bridgeBacktrackEndReasonName(
                                   bridge_correction.b.backtrack_end_reason)
                            << " end_query_s="
                            << bridge_correction.a.end_query_s << "/"
                            << bridge_correction.b.end_query_s
                            << " end_match_s="
                            << bridge_correction.a.end_matched_other_s << "/"
                            << bridge_correction.b.end_matched_other_s
                            << " end_match_distance="
                            << bridge_correction.a.end_match_distance << "/"
                            << bridge_correction.b.end_match_distance
                            << " end_direction_dot="
                            << bridge_correction.a.end_direction_dot << "/"
                            << bridge_correction.b.end_direction_dot
                            << " nearest_evaluations="
                            << bridge_correction.a.nearest_search_evaluations
                            << "/"
                            << bridge_correction.b.nearest_search_evaluations
                            << " geometric_attempted="
                            << (bridge_correction.a.
                                    geometric_extension_attempted
                                    ? "true" : "false")
                            << "/"
                            << (bridge_correction.b.
                                    geometric_extension_attempted
                                    ? "true" : "false")
                            << " geometric_applied="
                            << (bridge_correction.a.geometric_extension_applied
                                    ? "true" : "false")
                            << "/"
                            << (bridge_correction.b.geometric_extension_applied
                                    ? "true" : "false")
                            << " geometric_samples="
                            << bridge_correction.a.geometric_outer_samples
                            << "/"
                            << bridge_correction.b.geometric_outer_samples
                            << " geometric_overlap_samples="
                            << bridge_correction.a.geometric_overlap_samples
                            << "/"
                            << bridge_correction.b.geometric_overlap_samples
                            << " geometric_end_reason="
                            << bridgeGeometricEndReasonName(
                                   bridge_correction.a.geometric_end_reason)
                            << "/"
                            << bridgeGeometricEndReasonName(
                                   bridge_correction.b.geometric_end_reason)
                            << " geometric_end_query_s="
                            << bridge_correction.a.geometric_end_query_s
                            << "/"
                            << bridge_correction.b.geometric_end_query_s
                            << " geometric_matched_other_s="
                            << bridge_correction.a.
                                   geometric_end_matched_other_s
                            << "/"
                            << bridge_correction.b.
                                   geometric_end_matched_other_s
                            << " cusp_near_lost="
                            << (bridge_correction.a.cusp_near_relation_loss
                                    ? "true" : "false")
                            << "/"
                            << (bridge_correction.b.cusp_near_relation_loss
                                    ? "true" : "false");
                        coord_log_sink_(bridge_line.str());
                    }

                    recordConflictZones(
                        a, b, std::vector<ConflictZone>{zone},
                        ConflictMarkerKind::CROSSING_OR_OPPOSING,
                        event.first_overlap_t, -1, -1, 0.0,
                        VehicleAction::NOMINAL,
                        preferred_winner,
                        preferred_winner == a.id
                            ? b.id
                            : preferred_winner == b.id ? a.id : -1,
                        decimateTimedOverlaps(event.timed_overlaps),
                        interaction.type, event.last_t);
                    annotateTimedCollisionStartMarker();
                    annotateBridgeMarker();
                    continue;
                } else {
                    if (a1_related) ++dynamic_speed_metrics_.a1_fallbacks;
                    if (coord_log_sink_) {
                        std::ostringstream line;
                        line << std::fixed << std::setprecision(3)
                             << "[DYN-SPEED] pair=V" << key.first << "-V"
                             << key.second
                             << " first_overlap_t=" << event.first_overlap_t
                             << " selection=SKIPPED reason="
                             << (a1_related ? "a1_protected"
                                            : "legacy_special_case")
                             << " reservation=legacy";
                        coord_log_sink_(line.str());
                    }
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
                     << " first_overlap_t=" << event.first_overlap_t;
                coord_log_sink_(line.str());
            }

            recordConflictZones(
                a, b, std::vector<ConflictZone>{zone},
                ConflictMarkerKind::CROSSING_OR_OPPOSING,
                event.first_overlap_t,
                -1, -1, 0.0, VehicleAction::NOMINAL, holder,
                holder == a.id ? b.id : (holder == b.id ? a.id : -1),
                decimateTimedOverlaps(event.timed_overlaps));
            annotateTimedCollisionStartMarker();
            if (holder < 0) {
                brakeBefore(a, zone.s_self_enter, b.id);
                brakeBefore(b, zone.s_other_enter, a.id);
                if (a.reason == "time_brake_V" + std::to_string(b.id)) {
                    if (a1_.shouldLogDecision(a, b.id)) {
                        a1_.logDecision(a, &b, b.id);
                    }
                }
                if (b.reason == "time_brake_V" + std::to_string(a.id)) {
                    if (a1_.shouldLogDecision(b, a.id)) {
                        a1_.logDecision(b, &a, a.id);
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
            r.first_conflict_t = event.first_overlap_t;
            // Reaching the ownership path is now possible only for an A1
            // service transaction; every non-A1 timed conflict returned from
            // the rolling motion branch above.
            r.create_reason = "a1_related";
            ++dynamic_speed_metrics_.reservation_create_a1;
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
                    if (a1_.shouldLogDecision(b, a.id)) {
                        a1_.logDecision(b, &a, a.id);
                    }
                }
            } else {
                brakeBefore(a, zone.s_self_enter, b.id);
                if (a.reason == "time_brake_V" + std::to_string(b.id)) {
                    if (a1_.shouldLogDecision(a, b.id)) {
                        a1_.logDecision(a, &b, b.id);
                    }
                }
            }
        }
    }
}

void RuleEngine::enforceFutureA1Admission(
    std::vector<VehicleAgent>& vehicles, double dt) {
    const auto compute_full =
        [this](const VehicleAgent& first, const VehicleAgent& second) {
            return computeConflictZonesFull(first, second);
        };
    const auto canonical =
        [this](const VehicleAgent& first, const VehicleAgent& second) {
            const VehicleAgent& lo = first.id < second.id ? first : second;
            const VehicleAgent& hi = first.id < second.id ? second : first;
            const auto& cached = conflictBlocksCanonical(lo, hi);
            return std::vector<ConflictZone>(cached.begin(), cached.end());
        };
    const auto requests = a1_.enforceFutureAdmission(
        vehicles, dt, compute_full, canonical);
    for (const A1ActionRequest& request : requests) {
        for (VehicleAgent& vehicle : vehicles) {
            if (vehicle.id == request.vehicle_id) {
                applyActionRequest(vehicle, request.action, request.reason,
                                   request.blocker_id);
                break;
            }
        }
    }
    return;
}

void RuleEngine::resolveFollowing(std::vector<VehicleAgent>& vehicles) {
    // 璺熻溅鍙瘑鍒敮涓€鐨勭旱鍚?leader/follower 骞朵骇鐢熶綆浼樺厛绾у缓璁€?
    // 瀹冧笉鍐嶈烦杩?timed OBB銆佷笉鍒犻櫎 reservation銆佷笉鍙備笌 holder 閫夋嫨銆?
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
            // 闈欐€佸啿绐佸潡鏂瑰悜浠呬綔涓洪檮鍔犲嚑浣曟潯浠讹紱褰撳墠鐪熷疄杩愬姩鏂瑰悜宸插湪涓婃柟
            // 鐙珛鏍￠獙锛岄伩鍏嶅潡涓偣鏂瑰悜涓庡綋鍓嶅眬閮ㄦ柟鍚戜笉涓€鑷淬€?
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

            // 纭 v 鍦ㄥ悓鍚戝眬閮ㄨ溅閬撲笂璺熼殢 other銆傛闆嗗悎鍙敤浜庤瘖鏂紝
            // resolvePairwiseConflicts 浠嶄細瀹屾暣鎵ц銆?
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

void RuleEngine::enforceForwardClearance(std::vector<VehicleAgent>& vehicles,
                                         double dt) {
    // 鏅€傚墠鍚戝噣绌烘姢鏍?搂11.13.1 鍑哄彛妫€鏌ョ簿绁?+ 琛?following/crossing 鍒嗙被鎺ョ紳婕忔礊)銆?
    // 鎺ョ紳 bug:杩戜箮鍚屽悜銆佸悜涓嶅悓搴撲綅姹囪仛鐨勪袱杞?琚?pairwise 褰撹窡杞﹁烦杩囥€佸張涓嶆弧瓒?
    // resolveFollowing 鐨勬í鍚?闂磋窛鏉′欢 鈫?涓ゅ閮芥病鍒?鈫?鍚庤溅 NOMINAL 鐩存挒鍋滃湪璺彛鐨勫墠杞?
    // 鈫?褰㈡垚璋佸墠杩涢兘鎾炲鏂圭殑鍗佸瓧妤旀(canStepForward 鍙?false銆佺牬鐜棤鏁堛€佺‖鎶ゆ爮鍙嶅)銆?
    // 鍏滃簳:浠讳綍杞︽部鑷韩鍥哄畾璺緞鍦ㄣ€岃嚜宸卞埞杞﹁窛绂?+ 杞﹀ご鍓嶄几銆嶅唴浼氬帇涓婂彟涓€杈嗚溅鐨勫綋鍓?
    // 杞﹁韩,灏辨彁鍓?STOP銆備笉璁鸿鍝閫昏緫澶勭悊杩欓亾閮藉湪銆傛瘮纭姢鏍?0 浣欓噺銆佽创姝绘墠鍋?鏃╁埞銆?
    // 鐣欎綑閲?鈫?涓よ溅骞插噣瀵瑰仠鑰岄潪閲嶅彔 鈫?鐮寸幆杞?canStepForward 鑳藉垽鍑鸿皝鍙蛋 鈫?瑙ｅ紑銆?
    auto bodyAt = [&](const VehicleAgent& v, double s) {
        return makeBody(v.track.poseAtS(s), mp_, 0.0);
    };
    auto nextS = [&](const VehicleAgent& v) {
        if (!v.active()) {
            return v.mode == VehicleMode::DWELL
                ? v.track.length() : v.path_s;
        }
        const double desired = speedForAction(v.requested_action);
        double next_speed = std::max(0.0, v.current_speed);
        if (desired > next_speed) {
            next_speed = std::min(
                desired, next_speed + cfg_.max_accel * dt);
        } else {
            next_speed = std::max(
                desired, next_speed - cfg_.max_decel * dt);
        }
        return std::min(v.track.length(), v.path_s + next_speed * dt);
    };
    for (VehicleAgent& v : vehicles) {
        if (!v.active()) continue;
        if (v.deadlock_breaker) continue;  // 鐮寸幆杞﹁眮鍏?瀹冩琚巿鏉冨啿鍑虹幆
        if (v.requested_action == VehicleAction::STOP) continue;
        // 鍓嶆帰璺濈蹇呴』瓒冲杩?璁╄溅銆屾棭鏃╁仠鍦ㄥ啿绐佸尯澶栥€佺暀鍑洪棿闅欍€?鑰屼笉鏄啿鍒拌创涓婃墠鍒?
        // (浣庨€熸椂 brake_dist 鏋佸皬,鍙畻瀹冧細涓€鐩磋弓鍒版帴瑙︽墠鍋?妤旀)銆傛晠鍦ㄥ埞杞﹁窛绂讳箣澶?
        // 鍐嶅姞:杞﹀ご鍓嶄几 + 涓€涓浐瀹氬畨鍏ㄩ棿闅?kStandoff銆俴Standoff 鍚屾椂鏄€屽共鍑€瀵瑰仠銆嶅悗
        // 涓よ溅涔嬮棿鐣欏嚭鐨勪綑閲?浣跨牬鐜溅 canStepForward 鏈夌┖闂村垽鍒皝鑳藉姩銆傚彟璁炬渶灏忓墠鎺?
        // kMinLook,淇濊瘉鍗充究鍋滅潃(brake_dist鈮?)涔熻兘鐪嬪埌杩戝宸叉尅鍦ㄤ氦鍙夊彛鐨勮溅銆?
        const double s_end = nextS(v);
        int block_id = -1;
        const OBB body = bodyAt(v, s_end);
        for (const VehicleAgent& o : vehicles) {
            if (o.id == v.id) continue;
            if (o.mode == VehicleMode::NEED_TASK || o.track.empty()) continue;
            if (overlaps(body, bodyAt(o, nextS(o)))) {
                block_id = o.id;
                break;
            }
        }
        if (block_id >= 0) {
            const std::pair<int, int> key{
                std::min(v.id, block_id), std::max(v.id, block_id)};
            const int committed_frames = std::max(
                1, static_cast<int>(std::ceil(
                       cfg_.rolling_refresh_period / std::max(1e-6, dt))));
            const bool executable_frame =
                debug_log_source_ != "ROLLOUT" ||
                (debug_log_frame_id_ >= 0 &&
                 debug_log_frame_id_ < committed_frames);
            if (executable_frame &&
                ordinary_dynamic_pairs_.count(key) != 0 &&
                v.requested_action != VehicleAction::STOP) {
                ++dynamic_speed_metrics_.duplicate_pair_authority_overrides;
            }
            applyActionRequest(v, VehicleAction::STOP,
                               "emergency_next_step_V" +
                                   std::to_string(block_id),
                               block_id);
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
            v.ttc_stop_hold_remaining = 0.0;
            continue;
        }

        // 姝婚攣鎵撶牬璞佸厤鍊掕鏃讹細璞佸厤鏈熷唴涓嶅厑璁歌鍒欏眰灏嗚溅杈嗛檷涓?STOP
        if (v.cycle_break_immunity > 0.0) {
            v.cycle_break_immunity = std::max(0.0, v.cycle_break_immunity - dt);
        }

        const VehicleAction prev = v.action;            // last cycle's output
        VehicleAction req = v.requested_action;         // this cycle's rules

        if (v.ttc_stop_hold_remaining > 1e-9) {
            req = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.ttc_stop_hold_remaining = std::max(
                0.0, v.ttc_stop_hold_remaining - dt);
            if (v.reason == "clear") v.reason = "ttc_stop_hold";
        }

        // 姝婚攣璞佸厤锛氫粎褰?blocker 纭疄鏄閿佺幆鎴愬憳锛堢瓑寰呴摼鏈€缁堟寚鍥?v锛?
        // 鏃舵墠闄嶄负 CREEP锛涜嫢 blocker 鍙槸鎭板ソ鍋滅潃绛夌涓夋柟锛堥潪姝婚攣锛夛紝
        // 瀹夊叏绗竴锛屼繚鎸?STOP锛屼笉鑳芥湞鍋滅潃鐨勮溅 CREEP 杩囧幓銆?
        if (v.ttc_stop_hold_remaining <= 1e-9 &&
            v.cycle_break_immunity > 0.0 && req == VehicleAction::STOP) {
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
                req = VehicleAction::CREEP;  // 楠岃瘉涓烘閿佺幆锛岀紦琛屾墦鐮?
            } else {
                // blocker 涓嶆槸姝婚攣鐜垚鍛橈紝瀹夊叏绗竴
                v.cycle_break_immunity = 0.0;
            }
        }

        // 搂9 鐮寸幆杞︿繚搴?鍗充究鏌愬眰浠嶈瀹?STOP,涔熷己鍒惰嚦灏?CREEP 鍐插嚭鐜?瀹冨凡鍦ㄨ祫婧?
        // 浼樺厛绾у眰鎷垮埌鏈€楂樹紭鍏堢骇,杩欓噷淇濊瘉鍔ㄤ綔钀藉湴)銆傜‖鎶ゆ爮浠嶅厹搴曢槻鐪熺鎾炪€?
        if (v.ttc_stop_hold_remaining <= 1e-9 &&
            v.deadlock_breaker && req == VehicleAction::STOP) {
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
                r.cycle_break_immunity = 0.6;  // 0.6s 鍐呰鍒欏眰涓嶅緱灏嗚杞﹂噸鏂伴檷涓?STOP
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

    // 璧勬簮鐢宠绐楀彛(搂15 绗簩绫?涓嶈鎻愬墠鑰佽繙鍗犵敤璧勬簮)銆傚彧鏈夊綋杞﹀凡閫艰繎璧勬簮鍒?
    // 銆屽繀椤诲紑濮嬪喅绛栬兘鍚﹀湪鍋滄绾垮墠鍋滀綇銆嶇殑棰勮鐐规椂鎵嶈姹?棰勭害;绂诲緱杩滅殑璧勬簮涓嶉绾?
    // 浠ュ厤涓€杈嗚繕鍦ㄨ€佽繙鐨勯珮浼樺厛绾ц溅鎶婅祫婧愰攣姝汇€佽杩戝杞︽棤璋撳共绛?"鏄庢槑娌″暐浜嬪氨涓嶈蛋")銆?
    // 绐楀彛 = 鏍囩О閫熷埞鍋滆窛绂?+ 2 涓溅韬?鍐崇瓥缂撳啿),绾嚑浣曟淳鐢?闈炴媿鑴戣甯告暟銆?
    const double request_window =
        nominal * nominal / (2.0 * decel) + 2.0 * mp_.vehicle_length;

    // 1) 鎸夎祫婧愯仛鍚堣姹傝€?active 杞︾殑鍥哄畾璺緞浼氱敤鍒拌 capacity=1 浜掓枼璧勬簮
    //    (绐勯亾/璺彛/璐т綅鍙?銆佽溅灏捐繕娌℃暣杞﹂┒鍑恒€佷笖宸茶繘鍏ョ敵璇风獥鍙ｃ€?
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
            if (v.path_s - rear > sp.s_exit + 1e-6) continue;  // 宸叉暣杞﹂┒鍑?
            if (sp.s_enter - v.path_s > request_window) continue;  // 灏氳繙,涓嶉绾?
            by_res[sp.resource_id].push_back(Req{i, sp.s_enter, sp.s_exit});
        }
    }

    auto bodyInside = [&](const VehicleAgent& v, const Req& q) {
        return v.path_s + front >= q.s_enter - 1e-6 &&
               v.path_s - rear <= q.s_exit + 1e-6;
    };

    // 鏌愯溅褰撳墠杞﹁韩(鍚庤酱杩樺師杞﹁韩涓績)銆侱WELL 鐢ㄧ粓鐐逛綅濮?绌鸿建杩硅烦杩囥€?
    auto poseOf = [&](const VehicleAgent& o) {
        const double s = (o.mode == VehicleMode::DWELL) ? o.track.length()
                                                        : o.path_s;
        return o.track.poseAtS(std::min(s, o.track.length()));
    };
    // 鍑哄彛妫€鏌?搂11.4/搂11.13.1):鍊欓€夎溅椹跺嚭璧勬簮鍚庣殑钀借剼澶?s_exit + 鍗婅溅闀?
    // 鏄惁琚埆鐨勮溅韬崰浣忋€傝鍗?鈫?杩涘幓灏变細鍗″湪璧勬簮閲?鈫?涓嶅彂浠ょ墝(闄ら潪瀹冨凡鍦ㄥ尯鍐?
    // 蹇呴』椹跺畬)銆傝繖鏉滅粷"浠ょ墝鍙戠粰杩涘緱鍘诲嚭涓嶆潵鐨勮溅銆佸畠鏀ョ潃浠ょ墝鍗村姩涓嶄簡鎸℃鎵€鏈変汉"銆?
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
    // 鍙巿浜?= 宸插湪鍖哄唴(蹇呴』椹跺畬)鎴?鍑哄彛鐣呴€氥€?
    auto grantableK = [&](const std::vector<Req>& rs, size_t k) {
        return bodyInside(vehicles[rs[k].idx], rs[k]) || canExit(rs[k].idx, rs[k]);
    };

    // 2) 閫愯祫婧愪徊瑁?宸叉寔浠ょ墝涓斾粛鍦ㄨ姹?鈫?淇濇寔(鎸佹潈鍒版暣杞﹂┒鍑?闃茬炕杞?搂11);
    //    鍚﹀垯鎸夌粺涓€浼樺厛绾?PriorityKey 閫?winner 骞跺彂浠ょ墝(搂11.2)銆?
    for (auto& kv : by_res) {
        const int rid = kv.first;
        std::vector<Req>& reqs = kv.second;

        // 鎸佹湁鑰呬繚鎸佷护鐗岀殑鍓嶆彁:瀹冧粛鍦ㄨ姹?涓?浠嶅彲鎺堜簣(鑳介┒鍑?宸插湪鍖哄唴)銆?
        // 鍦ㄣ€屽彲鎺堜簣銆嶅€欓€夐噷鎸夌粺涓€ PriorityKey 閫?winner(搂11.2)銆傛寔浠ょ墝鑰呭甫
        // already_has_token 鍔犳垚(闃茬炕杞?搂11),鐮寸幆杞﹀甫 emergency 鍔犳垚(涓存椂鏈€楂?搂9)
        // 鈥斺€?浜庢槸 鐮寸幆杞?> 鎸佷护鐗岃€?> 鍏朵粬,涓€濂椾紭鍏堢骇缁熶竴瑁佸喅,涓嶅啀鐢?holder 鎹峰緞銆?
        const int holder = tokens_.holder(rid);
        int winner_k = -1;
        {
            ResourceRequest best;
            bool has_best = false;
            for (size_t k = 0; k < reqs.size(); ++k) {
                if (!grantableK(reqs, k)) continue;  // 鍑哄彛琚牭鐨勪笉鍙備笌(杩涘幓浼氬崱姝?
                const VehicleAgent& v = vehicles[reqs[k].idx];
                ResourceRequest rq;
                rq.vehicle_id = v.id;
                rq.wait_time = v.wait_time;
                rq.loaded = v.loaded;
                rq.task_count = v.task_count;
                rq.already_inside = bodyInside(v, reqs[k]);
                rq.already_has_token = (v.id == holder);
                rq.emergency_or_clear = v.deadlock_breaker;  // 搂9 鐮寸幆杞︿复鏃舵渶楂?
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
        // 鏃犱汉鍙巿浜?閮藉嚭涓嶅幓)鈫?閲婃斁浠ょ墝,涓斾笅闈㈣鎵€鏈夐€艰繎鑰呭仠鍦ㄤ笂娓?璋侀兘鍒繘)銆?
        const int winner_id = (winner_k >= 0) ? vehicles[reqs[winner_k].idx].id : -1;
        if (winner_id >= 0) {
            tokens_.grant(rid, winner_id, now_);  // 鎺堜簣/鍒锋柊(闃茶秴鏃?
        } else {
            tokens_.release(rid);
        }

        // 3) 闈?winner(鍙婃棤浜哄彲鎺堜簣鏃剁殑鍏ㄩ儴閫艰繎鑰?:鍦ㄤ笂娓稿仠姝㈢嚎璁╄銆傚彧鍦ㄣ€屾寜褰撳墠
        //    閫熷害+max_decel 鍗冲皢鍒逛笉浣忋€嶆椂鎵嶅彂 STOP(搂10 鍋滄绾?搂15 涓嶈繃鏃╃瓑鈥斺€旂寰?
        //    杩滃氨缁х画鎺ヨ繎,涓嶅師鍦板共绛?銆?
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
        // track 闀垮害鍙樺寲鍗宠涓烘崲浜嗘柊璺緞(鏂颁换鍔?,閲嶇畻璧勬簮鍗犵敤鍖洪棿銆?
        if (std::abs(v.track.length() - v.spans_track_len) > 1e-6) {
            v.resource_spans = resmap_->spansForPath(v.track);
            v.spans_track_len = v.track.length();
        }
    }
}

void RuleEngine::resolveDeadlock(std::vector<VehicleAgent>& vehicles, double dt) {
    // 搂9/搂11.11:鐢ㄤ笂涓€鍛ㄦ湡娈嬬暀鐨勭瓑寰呰竟(blocker_id,鏈懆鏈?reset 鍓嶄粛鏈夋晥)寤虹瓑寰呭浘,
    // 妫€娴嬬幆,閫夌牬鐜溅缁欎复鏃舵渶楂樹紭鍏堢骇(deadlock_breaker)銆傜瓑寰呭浘鏄姛鑳藉浘(姣忚溅鑷冲
    // 涓€鏉″嚭杈?瀹冨湪绛夌殑 blocker),璺熸寚閽堝嵆鍙壘鐜€?
    constexpr double kBreakerHold = 2.0;  // 鐮寸幆韬唤杩熸粸淇濇寔绉掓暟(闃查棯鐑佽弓琛?
    const size_t n = vehicles.size();
    // 杩熸粸:淇濇寔鏈熸湭鍒扮殑鐮寸幆杞︾户缁槸鐮寸幆杞?瀹冧竴鍔ㄨ捣鏉ュ氨涓嶅啀 STOP 绛夊緟銆佺幆妫€娴嬩細娑堝け,
    // 鑻ヤ笉淇濇寔灏变細鏍囧織闂儊鈫掕鏃у眰鍙嶅鎽佸仠鈫掕弓琛?銆備繚鎸佹湡鍒颁簡鎵嶆竻銆?
    for (VehicleAgent& v : vehicles) {
        v.deadlock_breaker_hold = std::max(0.0, v.deadlock_breaker_hold - dt);
        v.deadlock_breaker = (v.deadlock_breaker_hold > 0.0);
    }

    auto indexOfId = [&](int id) -> int {
        for (size_t i = 0; i < n; ++i)
            if (vehicles[i].id == id) return static_cast<int>(i);
        return -1;
    };
    // 鐮寸幆杞﹀繀椤婚€夈€岀湡姝ｈ兘寰€鍓嶈蛋鐨勯偅杈嗐€?搂16):鐢ㄨ溅韬嚑浣曞疄娴嬧€斺€旇溅 i 娌垮浐瀹氳矾寰?
    // 鍓嶈繘涓€灏忔(probe),杞﹁韩鏄惁浠嶄笌浠讳綍鍏跺畠杞︾殑褰撳墠杞﹁韩涓嶉噸鍙犮€傝兘=瀹冨墠杩涘彲鑴卞洶銆?
    // 鎶栧紑鐜?涓嶈兘(鍓嶆柟灏辨槸瀵瑰啿杞?=鏀惧畠涔熸槸琚‖鎶ゆ爮鎽佹銆佺幆姘歌繙鐮翠笉浜嗐€傝繖淇浜?
    // 鏃х殑 min-id 鐩查€?妤旀鏃跺父鎶娿€屽姩涓嶄簡鐨勯偅杈嗐€嶉€夋垚鐮寸幆杞﹁€屽緬鍔炽€?
    auto bodyAtCurrent = [&](const VehicleAgent& v) {
        const double s =
            (v.mode == VehicleMode::DWELL) ? v.track.length() : v.path_s;
        return makeBody(v.track.poseAtS(s), mp_, 0.0);
    };
    auto canStepForward = [&](int i) -> bool {
        const VehicleAgent& v = vehicles[i];
        if (v.track.empty()) return false;
        constexpr double kProbe = 0.10;  // 鍓嶆帰绾﹀崐韬?瓒充互鍒ゅ埆鍓嶆柟鏄惁琚鍐茶溅鍫垫
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
        if (v.action != VehicleAction::STOP) return -1;  // 鍙湅瀹屽叏鍋滀綇鐨勭瓑寰?
        if (v.blocker_id < 0) return -1;
        return indexOfId(v.blocker_id);
    };

    std::vector<int> state(n, 0);  // 0 鏈 1 鍦ㄦ爤 2 瀹屾垚
    for (size_t s = 0; s < n; ++s) {
        if (state[s] != 0) continue;
        std::vector<int> path;
        int cur = static_cast<int>(s);
        while (cur >= 0 && state[cur] == 0) {
            state[cur] = 1;
            path.push_back(cur);
            cur = waitEdge(cur);
        }
        if (cur >= 0 && state[cur] == 1) {  // 鎵惧埌鐜?浠?cur 棣栨鍑虹幇鍒版湯灏?
            const auto begin = std::find(path.begin(), path.end(), cur);
            // 鍗忚皟鍥剧5姝?涓ユ牸鍏ㄥ簭(unifiedPriority)浣?璋佽璋?鍏崇郴鏈簲鏃犵幆 鈬?杩欓噷妫€娴嬪埌
            // 鐜?鏃犵幆淇濊瘉琚牬鍧忕殑 bug 淇″彿(閫氬父鏄?priorityWinner 閲岀殑 slot 璧勬簮鍓嶇疆绾︽潫
            // 鎴愮幆,鍗充换鍔″垎閰嶅嚭鐜板惊鐜崰浣?銆傞檷涓烘柇瑷€:**鍛婅**鎶婂畠鏆撮湶鍑烘潵銆傜牬鐜€冪敓鏆傜暀
            // 浣滃畨鍏ㄧ綉(纭闀胯窇涓嶅啀鍛婅鍚庡彲绉婚櫎)銆?
            {
                std::string ring;
                for (auto p = begin; p != path.end(); ++p) {
                    if (p != begin) ring += "->";
                    ring += "V" + std::to_string(vehicles[*p].id);
                }
                ROS_WARN_THROTTLE(
                    1.0,
                    "[DIAG cycle] wait graph cycle=%s; using the existing "
                    "deadlock escape fallback.",
                    ring.c_str());
            }
            // 閫夌牬鐜溅:浼樺厛銆屽墠杩涗竴姝ヨ兘鑴卞洶銆嶇殑杞?canStepForward),鍚岀被閲屽彇 id 鏈€灏?
            // (纭畾鎬?搂19)銆傝嫢鐜唴鏃犱汉鑳藉墠杩?鐪熉锋姝?闇€鍊掕溅鐨勭綍瑙佹儏褰?,閫€鍥?id
            // 鏈€灏忋€傚彧鏀句竴杈?鍏朵綑姝ｅ父璁╄鍋滀綇鈥斺€旂粷涓嶄細涓よ締瀵瑰啿杞︿竴璧疯鏀捐€岀浉鎾炪€?
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
            if (breaker < 0) breaker = breaker_any;  // 鏃犱汉鑳藉墠杩涒啋閫€鍥?min-id
            // 鐮寸幆閫冪敓宸插仠鐢?闄嶄负绾娴?鍛婅):鍦ㄣ€岀姝㈠€掕溅銆嶅墠鎻愪笅,鍓嶅悜鐮寸幆鏈韩鏄鎾炴簮
            // 鈥斺€斿畠璞佸厤璁╄鏂瑰埞杞︺€佸己鎺ㄥ畠鍓嶅啿鑴卞洶,鑰屽墠鏂瑰氨鏄幆閲屽埆鐨勮溅 鈫?鐩存帴椤朵笂鍘?瀹炴祴
            // V2 brkr=1 琚己鎺ㄩ《杩涘崱姝荤殑 V7,姝ゅ悗涓よ溅璐存銆佺‖鎶ゆ爮姣忔媿瑙﹀彂=閭?16857 娆?纰版挒")銆?
            // 鏃㈢劧涓嶈兘鍊掕溅,鐜氨搴旀槸銆岄潤姝㈠宄?妤旀)銆嶈€岄潪銆屽鎾炪€?瀹夊叏浼樺厛,鍏堟妸纰版挒闄嶇骇銆?
            // 鐪熸鍑鸿矾鏄粠婧愬ご涓嶈鐜舰鎴?鍑哄簱鍙?鍑哄彛妫€鏌?,瑙?鑽夊饱铏鍒檁鍗忚皟鍥剧粺涓€鏋舵瀯璁捐銆?
            (void)breaker; (void)kBreakerHold;
            // vehicles[breaker].deadlock_breaker = true;          // 宸插仠鐢?
            // vehicles[breaker].deadlock_breaker_hold = kBreakerHold;
        }
        for (int p : path) state[p] = 2;
    }
}

void RuleEngine::decide(std::vector<VehicleAgent>& vehicles, double dt,
                        double prediction_horizon_override,
                        bool reuse_ordinary_coordination,
                        const RollingDynamicDecision*
                            period_ordinary_decision) {
    conflicts_.clear();
    last_rolling_dynamic_decision_ = RollingDynamicDecision{};
    now_ += dt;                      // 鍐呴儴浠跨湡鏃堕挓(浠ょ墝闃叉姈/瓒呮椂)
    resolveDeadlock(vehicles, dt);   // Phase4:鐢ㄤ笂鍛ㄦ湡绛夊緟杈规娴嬬幆銆侀€夌牬鐜溅(reset 鍓?
    refreshResourceSpans(vehicles);  // Phase 2:鍒锋柊姣忚溅璺緞鐨勮祫婧愬崰鐢ㄧ紦瀛?

    previous_following_followers_.clear();
    previous_dynamic_actions_.clear();
    for (const VehicleAgent& v : vehicles) {
        if (v.blocker_id < 0) continue;
        if (!reuse_ordinary_coordination &&
            v.reason.rfind("dynamic_speed_", 0) == 0 &&
                (v.action == VehicleAction::YIELD ||
                 v.action == VehicleAction::CREEP)) {
            const std::pair<int, int> key{std::min(v.id, v.blocker_id),
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
        if (v.mode != VehicleMode::ACTIVE) {
            v.blocker_id = -1;
            v.cycle_break_immunity = 0.0;
            v.requested_action = VehicleAction::STOP;
            v.reason = "not_active";
            v.ttc_stop_hold_remaining = 0.0;
            continue;
        }
        if (v.ttc_stop_hold_remaining > 1e-9) {
            v.requested_action = VehicleAction::STOP;
            if (v.reason.rfind("dynamic_speed_STOP_", 0) != 0) {
                v.reason = "ttc_stop_hold";
            }
            continue;
        }
        v.blocker_id = -1;
        v.requested_action = VehicleAction::NOMINAL;
        v.reason = "clear";
        if (reuse_ordinary_coordination &&
            period_ordinary_decision != nullptr) {
            for (const RollingDynamicDecision::Target& target :
                 period_ordinary_decision->targets) {
                if (target.vehicle_id != v.id ||
                    target.path_gen != v.path_gen) {
                    continue;
                }
                v.requested_action = target.action;
                v.blocker_id = target.blocker_id;
                v.reason = target.reason;
                break;
            }
        }
    }

    // 娣卞眰鏍规不(鐢ㄦ埛娲炲療:璺緞鍥哄畾鈫掑彧淇＄簿纭嚑浣?:鍋滅敤绮楃矑搴﹁祫婧愮洅浠茶(璺彛/杞﹂亾
    // 浠ょ墝),瀹冩妸"鍏辩敤涓€涓矾鍙ｇ洅"褰撳啿绐侀€犳垚骞昏薄鍐茬獊鈫掓墦鏋垛啋姝婚攣銆傛敼鐢辩簿纭殑 pairwise
    // 鍑犱綍鍐茬獊(findConflictZones:娌垮浐瀹氳矾寰勯噰鏍疯溅韬玂BB,鍙爣鐪熷疄閲嶅彔寮ф)浣滃敮涓€浜ゅ弶
    // 鍗忚皟鏉冨▉銆傚叓绔垮瓙鎵撲笉鐫€鐨勪袱杞﹀畠鏍规湰涓嶆姤鍐茬獊鈫掑悇鑷叏閫熴€?
    // arbitrateResources(vehicles, dt);   // 宸插仠鐢?璧勬簮鐩?骞昏薄鍐茬獊婧?
    const double pairwise_horizon = prediction_horizon_override >= 0.0
        ? prediction_horizon_override
        : cfg_.prediction_horizon;
    refreshDepartureClusterCommitments(vehicles);
    resolvePairwiseConflicts(vehicles, dt, pairwise_horizon,
                             reuse_ordinary_coordination);
    enforceFutureA1Admission(vehicles, dt);
    enforceDepartureClusterCommitments(vehicles, dt);
    resolveTargetSlotOccupancy(vehicles);  // slot-mouth queueing (spec 6/7)
    // Safety validation runs after all coordination/special-resource outputs
    // and may only reject a physically illegal next control step.
    enforceForwardClearance(vehicles, dt);
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
