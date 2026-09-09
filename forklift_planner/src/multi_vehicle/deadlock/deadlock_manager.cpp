#include "forklift_planner/multi_vehicle/deadlock/deadlock_manager.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <ros/ros.h>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

//得到一辆车当前用于几何检测的路径纵向位置s，若休眠则证明在库位，走完路径。
double vehiclePoseS(const VehicleAgent& vehicle) {
    return vehicle.mode == VehicleMode::DWELL
               ? vehicle.track.length()
               : vehicle.path_s;
}


template <typename Callback>
//在一段路径 s 区间上按照指定步长采样，并对每个采样点执行回调
bool sampleInterval(double begin, double end, double step,
                    const Callback& callback) {
    const double direction = end >= begin ? 1.0 : -1.0;     //路径方向
    const double distance = std::abs(end - begin);          //路径长度
    const int count = std::max(1, static_cast<int>(std::ceil(distance / step)));
    for (int i = 0; i <= count; ++i) {
        const double ratio = static_cast<double>(i) / count;
        if (!callback(begin + direction * distance * ratio)) return false;
    }
    return true;
}

}  // namespace

//Recovery 状态与动作解释，用于日志调试
const char* recoveryPhaseName(RecoveryPhase phase) {
    switch (phase) {
        case RecoveryPhase::NONE: return "NONE";
        case RecoveryPhase::RETREAT: return "RETREAT";
        case RecoveryPhase::PASS: return "PASS";
        case RecoveryPhase::CLEAR: return "CLEAR";
        case RecoveryPhase::UNRESOLVED: return "UNRESOLVED";
        case RecoveryPhase::ABORT: return "ABORT";
    }
    return "UNKNOWN";
}

//寻找当前死锁恢复状态下，某一辆车到底应该执行RETREAT、HOLD 还是 NORMAL
RecoveryMotion RecoveryDirective::motionFor(int vehicle_id) const {
    if (cooldownActive() && vehicle_id == cooldown_vehicle_id) {
        return RecoveryMotion::HOLD;
    }
    //在死锁解决触发的恢复状态下，retreat车辆执行退让，pass车辆执行hold静止
    if (phase == RecoveryPhase::RETREAT) {
        if (vehicle_id == retreat_vehicle_id) return RecoveryMotion::RETREAT;
        if (vehicle_id == pass_vehicle_id &&
            hold_pass_vehicle_during_retreat) {
            return RecoveryMotion::HOLD;
        }
    } else if (phase == RecoveryPhase::PASS) {
        if (vehicle_id == retreat_vehicle_id) return RecoveryMotion::HOLD;
    } else if (phase == RecoveryPhase::UNRESOLVED) {
        if (vehicle_id == retreat_vehicle_id || vehicle_id == pass_vehicle_id) {
            return RecoveryMotion::HOLD;
        }
    }
    return RecoveryMotion::NORMAL;
}

bool DeadlockManager::retreatPoseClearsPassCorridor(
    const VehicleAgent& retreat, const VehicleAgent& passer,
    double retreat_s, double pass_clear_s) const {
    if (retreat.track.empty() || passer.track.empty()) return false;
    const OBB stopped = makeBody(retreat.track.poseAtS(retreat_s),
                                 map_param_, 0.0);
    const double sweep_step = std::max(
        0.005, std::min(0.01, config_.path_validation_step));
    return sampleInterval(passer.path_s, pass_clear_s, sweep_step,
                          [&](double pass_s) {
        const OBB pass_body = makeBody(passer.track.poseAtS(pass_s),
                                       map_param_, 0.0);
        return !overlaps(stopped, pass_body);
    });
}

//构造函数。创建一个 DeadlockManager 对象时，这个构造函数会执行一次
DeadlockManager::DeadlockManager(const MapParam& map_param,
                                 const MultiVehicleConfig& config)
    //读取传入的 map_param 和 config，并分别保存到自己的 map_param_ 和 config_ 成员中。
    : map_param_(map_param), config_(config) {}  //成员初始化列表

//查询函数，输入id，在vehicles容器中查找对应车辆；找到就返回该车辆的只读指针，找不到就返回 nullptr
const VehicleAgent* DeadlockManager::vehicleById(
    const std::vector<VehicleAgent>& vehicles, int id) const {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.id == id) return &vehicle;
    }
    return nullptr;
}

//根据两个车辆 ID，在死锁车辆对几何信息表中查找对应的组队
//车辆顺序不敏感；找到返回指针，找不到返回空指针
const DeadlockPairGeometry* DeadlockManager::geometryFor(
    const std::vector<DeadlockPairGeometry>& geometry,
    int vehicle_a, int vehicle_b) const {
    for (const DeadlockPairGeometry& item : geometry) {
        if ((item.vehicle_a == vehicle_a && item.vehicle_b == vehicle_b) ||
            (item.vehicle_a == vehicle_b && item.vehicle_b == vehicle_a)) {
            return &item;
        }
    }
    return nullptr;
}

//检查 retreat 车辆从当前位置一路倒到 target_s 的整个车身扫掠区域，会不会撞到任何其他车辆。
//判断向后退，是否会撞到其他车
bool DeadlockManager::retreatSweepClear(
    const VehicleAgent& retreat, const VehicleAgent& passer,
    const std::vector<VehicleAgent>& vehicles, double target_s) const {
    //确定采样步长，在0.005m和0.01m之间
    const double sweep_step = std::max(
        0.005, std::min(0.01, config_.path_validation_step));
    //创建匿名函数，从当前 path_s 一直采样到 target_s，检查后退车辆沿途每一个采样位置是否会和其他车辆车身重叠；
    //只要有一个位置发生碰撞，就返回 false；全部位置都安全，返回 true
    return sampleInterval(retreat.path_s, target_s, sweep_step,
                          [&](double retreat_s) {
        const OBB body = makeBody(retreat.track.poseAtS(retreat_s),
                                  map_param_, 0.0);
        for (const VehicleAgent& other : vehicles) {
            if (other.id == retreat.id || other.track.empty() ||
                other.mode == VehicleMode::NEED_TASK) {
                continue;
            }
            const double other_s = other.id == passer.id
                                       ? passer.path_s
                                       : vehiclePoseS(other);
            const OBB obstacle = makeBody(other.track.poseAtS(other_s),
                                          map_param_, 0.0);
            if (overlaps(body, obstacle)) return false;
        }
        return true;
    });
}

DeadlockManager::RetreatEvaluation DeadlockManager::evaluateRetreat(
    const VehicleAgent& retreat, const VehicleAgent& passer,
    const std::vector<VehicleAgent>& vehicles) const {
    RetreatEvaluation result;
    result.retreat_vehicle_id = retreat.id;
    result.pass_vehicle_id = passer.id;
    if (retreat.track.empty() || passer.track.empty()) {
        result.reason = "empty_track";
        return result;
    }

    // NORMAL_DEADLOCK geometry is derived directly from the two bare-body
    // OBBs.  Build the overlap set in (pass_s, retreat_s), seed it with the
    // overlap sample nearest the current state, then retain only its
    // 8-connected component.  A later, disconnected crossing must not extend
    // this recovery transaction.
    constexpr double kClosureStep = 0.01;
    auto samples = [&](double begin, double end) {
        const double distance = std::max(0.0, end - begin);
        const size_t count = std::max<size_t>(
            1, static_cast<size_t>(std::ceil(distance / kClosureStep)));
        std::vector<double> values(count + 1);
        for (size_t i = 0; i <= count; ++i) {
            values[i] = begin + distance * static_cast<double>(i) /
                                    static_cast<double>(count);
        }
        return values;
    };
    const std::vector<double> pass_samples = samples(
        passer.path_s, passer.track.length());
    const std::vector<double> retreat_samples = samples(0.0, retreat.path_s);
    const size_t rows = pass_samples.size();
    const size_t cols = retreat_samples.size();
    if (rows == 0 || cols == 0 ||
        rows > std::numeric_limits<size_t>::max() / cols) {
        result.reason = "invalid_obb_closure_grid";
        return result;
    }

    std::vector<OBB> pass_bodies;
    std::vector<OBB> retreat_bodies;
    pass_bodies.reserve(rows);
    retreat_bodies.reserve(cols);
    for (double s : pass_samples) {
        pass_bodies.push_back(makeBody(passer.track.poseAtS(s),
                                       map_param_, 0.0));
    }
    for (double s : retreat_samples) {
        retreat_bodies.push_back(makeBody(retreat.track.poseAtS(s),
                                          map_param_, 0.0));
    }

    std::vector<uint8_t> overlap_grid(rows * cols, 0);
    size_t seed = rows * cols;
    double seed_distance_sq = std::numeric_limits<double>::infinity();
    for (size_t pass_i = 0; pass_i < rows; ++pass_i) {
        const double pass_delta = pass_samples[pass_i] - passer.path_s;
        for (size_t retreat_i = 0; retreat_i < cols; ++retreat_i) {
            if (!overlaps(pass_bodies[pass_i],
                          retreat_bodies[retreat_i])) {
                continue;
            }
            const size_t index = pass_i * cols + retreat_i;
            overlap_grid[index] = 1;
            const double retreat_delta =
                retreat.path_s - retreat_samples[retreat_i];
            const double distance_sq = pass_delta * pass_delta +
                                       retreat_delta * retreat_delta;
            if (distance_sq < seed_distance_sq) {
                seed_distance_sq = distance_sq;
                seed = index;
            }
        }
    }
    if (seed == overlap_grid.size()) {
        result.reason = "no_obb_overlap_component";
        return result;
    }

    size_t min_retreat_i = seed % cols;
    size_t max_pass_i = seed / cols;
    std::vector<size_t> frontier{seed};
    overlap_grid[seed] = 2;
    for (size_t head = 0; head < frontier.size(); ++head) {
        const size_t index = frontier[head];
        const size_t pass_i = index / cols;
        const size_t retreat_i = index % cols;
        min_retreat_i = std::min(min_retreat_i, retreat_i);
        max_pass_i = std::max(max_pass_i, pass_i);
        for (int dp = -1; dp <= 1; ++dp) {
            for (int dr = -1; dr <= 1; ++dr) {
                if (dp == 0 && dr == 0) continue;
                const std::ptrdiff_t next_pass =
                    static_cast<std::ptrdiff_t>(pass_i) + dp;
                const std::ptrdiff_t next_retreat =
                    static_cast<std::ptrdiff_t>(retreat_i) + dr;
                if (next_pass < 0 || next_retreat < 0 ||
                    next_pass >= static_cast<std::ptrdiff_t>(rows) ||
                    next_retreat >= static_cast<std::ptrdiff_t>(cols)) {
                    continue;
                }
                const size_t next = static_cast<size_t>(next_pass) * cols +
                                    static_cast<size_t>(next_retreat);
                if (overlap_grid[next] != 1) continue;
                overlap_grid[next] = 2;
                frontier.push_back(next);
            }
        }
    }

    result.pass_clear_s = pass_samples[max_pass_i];
    const double retreat_clear_boundary_s = retreat_samples[min_retreat_i];
    result.target_s = std::max(
        0.0, retreat_clear_boundary_s -
                 config_.deadlock_retreat_clearance);
    if (result.target_s >= retreat.path_s - 1e-9 ||
        !retreatPoseClearsPassCorridor(
            retreat, passer, result.target_s, result.pass_clear_s) ||
        !retreatSweepClear(retreat, passer, vehicles, result.target_s)) {
        result.reason = "retreat_sweep_or_corridor_blocked";
        return result;
    }

    result.feasible = true;
    result.distance = retreat.path_s - result.target_s;
    result.reason = "clear";
    return result;
}

void DeadlockManager::refreshDirective() {
    directive_ = {};
    directive_.phase = transaction_.phase;
    directive_.retreat_vehicle_id = transaction_.retreat_vehicle_id;
    directive_.pass_vehicle_id = transaction_.pass_vehicle_id;
    directive_.retreat_path_gen = transaction_.retreat_path_gen;
    directive_.pass_path_gen = transaction_.pass_path_gen;
    directive_.retreat_target_s = transaction_.retreat_target_s;
    directive_.pass_clear_s = transaction_.pass_clear_s;
    directive_.retreat_distance = transaction_.retreat_distance;
    directive_.estimated_retreat_time = transaction_.estimated_retreat_time;
    directive_.cooldown_vehicle_id = cooldown_.vehicle_id;
    directive_.cooldown_path_gen = cooldown_.path_gen;
    directive_.cooldown_remaining = cooldown_.remaining;
    directive_.hold_pass_vehicle_during_retreat =
        transaction_.hold_pass_vehicle_during_retreat;
    directive_.reason = transaction_.reason;
}

void DeadlockManager::emit(const char* event, const std::string& details,
                           bool enabled) const {
    if (!enabled) return;
    const std::string line =
        std::string("[DEADLOCK] event=") + event + " " + details;
    if (log_sink_) log_sink_(line);
    const std::string name(event);
    if (name == "CONFIRMED" || name == "SELECT" ||
        name == "RETREAT_DONE" || name == "PASS_START" ||
        name == "CLEAR" || name == "UNRESOLVED" || name == "ABORT") {
        ROS_WARN_STREAM(line);
    }
}

void DeadlockManager::abort(const std::string& reason, bool emit_logs) {
    const int retreat_id = transaction_.retreat_vehicle_id;
    const int pass_id = transaction_.pass_vehicle_id;
    emit("ABORT", "pair=V" + std::to_string(retreat_id) +
                      "-V" + std::to_string(pass_id) + " reason=" + reason,
         emit_logs);
    candidate_ = {};
    transaction_ = {};
    refreshDirective();
}

void DeadlockManager::clearSuccessfulRecovery(
    const VehicleAgent* retreat, const VehicleAgent* passer,
    const std::string& reason, bool emit_logs) {
    const int retreat_id = transaction_.retreat_vehicle_id;
    const int pass_id = transaction_.pass_vehicle_id;
    const double pass_s = passer != nullptr ? passer->path_s : 0.0;
    std::ostringstream details;
    details << "pair=V" << retreat_id << "-V" << pass_id
            << " pass_s=" << pass_s
            << " reason=" << reason;
    emit("CLEAR", details.str(), emit_logs);

    cooldown_.vehicle_id = retreat_id;
    cooldown_.path_gen = retreat != nullptr
        ? retreat->path_gen : transaction_.retreat_path_gen;
    cooldown_.remaining = config_.rolling_refresh_period;
    candidate_ = {};
    transaction_ = {};
    refreshDirective();
}

void DeadlockManager::update(
    const std::vector<VehicleAgent>& vehicles,
    const std::vector<DeadlockPairGeometry>& pair_geometry,
    double dt, bool emit_logs) {
    if (!config_.deadlock_enabled) {
        candidate_ = {};
        transaction_ = {};
        cooldown_ = {};
        directive_ = {};
        return;
    }

    if (cooldown_.remaining > 1e-9) {
        const VehicleAgent* cooling = vehicleById(vehicles,
                                                   cooldown_.vehicle_id);
        if (cooling == nullptr || cooling->mode != VehicleMode::ACTIVE ||
            cooling->path_gen != cooldown_.path_gen) {
            cooldown_ = {};
        } else {
            cooldown_.remaining = std::max(0.0, cooldown_.remaining - dt);
            if (cooldown_.remaining <= 1e-9) cooldown_ = {};
        }
        refreshDirective();
    }

    if (transaction_.phase != RecoveryPhase::NONE) {
        const VehicleAgent* retreat = vehicleById(
            vehicles, transaction_.retreat_vehicle_id);
        const VehicleAgent* passer = vehicleById(
            vehicles, transaction_.pass_vehicle_id);

        // PASS ends only after the passer has physically cleared the frozen
        // transaction corridor. Check success before identity changes so a
        // natural arrival at the end of the same path is not misclassified.
        if (transaction_.phase == RecoveryPhase::PASS) {
            if (passer == nullptr) {
                abort("pass_vehicle_missing", emit_logs);
                return;
            }
            transaction_.pass_confirmation_elapsed += std::max(0.0, dt);
            const bool same_path_cleared =
                passer->path_gen == transaction_.pass_path_gen &&
                passer->path_s + 1e-9 >= transaction_.pass_clear_s;
            const bool natural_path_completion =
                passer->path_gen != transaction_.pass_path_gen &&
                transaction_.pass_track_length + 1e-9 >=
                    transaction_.pass_clear_s;
            if (same_path_cleared || natural_path_completion) {
                clearSuccessfulRecovery(
                    retreat, passer, "passer_cleared_pass_corridor",
                    emit_logs);
                return;
            }
            if (retreat == nullptr ||
                retreat->mode != VehicleMode::ACTIVE ||
                retreat->path_gen != transaction_.retreat_path_gen ||
                passer->mode != VehicleMode::ACTIVE ||
                passer->path_gen != transaction_.pass_path_gen) {
                abort("vehicle_or_path_identity_changed", emit_logs);
            }
            return;
        }

        if (retreat == nullptr || passer == nullptr ||
            retreat->mode != VehicleMode::ACTIVE ||
            passer->mode != VehicleMode::ACTIVE ||
            retreat->path_gen != transaction_.retreat_path_gen ||
            passer->path_gen != transaction_.pass_path_gen) {
            abort("vehicle_or_path_identity_changed", emit_logs);
            return;
        }

        if (transaction_.phase == RecoveryPhase::RETREAT) {
            if (!retreatSweepClear(*retreat, *passer, vehicles,
                                   transaction_.retreat_target_s)) {
                abort("retreat_sweep_invalidated", emit_logs);
                return;
            }
            const double tolerance = std::max(
                0.005, 0.25 * config_.deadlock_retreat_search_step);
            if (retreat->path_s <= transaction_.retreat_target_s + tolerance) {
                if (!retreatPoseClearsPassCorridor(
                        *retreat, *passer, retreat->path_s,
                        transaction_.pass_clear_s)) {
                    transaction_.reason =
                        "actual_retreat_pose_still_blocks_pass_corridor";
                    refreshDirective();
                    return;
                }
                transaction_.phase = RecoveryPhase::PASS;
                transaction_.pass_confirmation_elapsed = 0.0;
                transaction_.reason = "actual_retreat_pose_clears_corridor";
                refreshDirective();
                std::ostringstream details;
                details << "pair=V" << retreat->id << "-V" << passer->id
                        << " retreat=V" << retreat->id
                        << " pass=V" << passer->id
                        << " target_s=" << transaction_.retreat_target_s;
                emit("RETREAT_DONE", details.str(), emit_logs);
                emit("PASS_START", details.str(), emit_logs);
            }
            return;
        }

        return;
    }

    // The manager owns one short-lived recovery at a time. During restart
    // hold, keep all other vehicles under ordinary coordination and wait
    // until this single-vehicle cooldown has expired before confirming a new
    // deadlock transaction.
    if (cooldown_.remaining > 1e-9) {
        candidate_ = {};
        return;
    }

    const VehicleAgent* candidate_a = nullptr;
    const VehicleAgent* candidate_b = nullptr;
    for (const VehicleAgent& a : vehicles) {
        if (a.mode != VehicleMode::ACTIVE ||
            a.action != VehicleAction::STOP || a.blocker_id < 0) {
            continue;
        }
        if (cooldown_.remaining > 1e-9 &&
            a.id == cooldown_.vehicle_id) continue;
        const VehicleAgent* b = vehicleById(vehicles, a.blocker_id);
        if (b == nullptr || b->mode != VehicleMode::ACTIVE ||
            b->action != VehicleAction::STOP || b->blocker_id != a.id) {
            continue;
        }
        if (cooldown_.remaining > 1e-9 &&
            b->id == cooldown_.vehicle_id) continue;
        if (a.id < b->id) {
            candidate_a = &a;
            candidate_b = b;
            break;
        }
    }

    if (candidate_a == nullptr) {
        candidate_ = {};
        refreshDirective();
        return;
    }

    const DeadlockPairGeometry* observed_geometry = geometryFor(
        pair_geometry, candidate_a->id, candidate_b->id);
    if (observed_geometry == nullptr) {
        candidate_ = {};
        directive_ = {};
        return;
    }

    const double progress_epsilon = std::max(
        0.005, 0.25 * config_.deadlock_retreat_search_step);
    const bool same_candidate = candidate_.valid &&
        candidate_.vehicle_a == candidate_a->id &&
        candidate_.vehicle_b == candidate_b->id &&
        candidate_.path_gen_a == candidate_a->path_gen &&
        candidate_.path_gen_b == candidate_b->path_gen;
    if (!same_candidate ||
        std::abs(candidate_a->path_s - candidate_.anchor_s_a) >
            progress_epsilon ||
        std::abs(candidate_b->path_s - candidate_.anchor_s_b) >
            progress_epsilon) {
        candidate_ = {};
        candidate_.valid = true;
        candidate_.vehicle_a = candidate_a->id;
        candidate_.vehicle_b = candidate_b->id;
        candidate_.path_gen_a = candidate_a->path_gen;
        candidate_.path_gen_b = candidate_b->path_gen;
        candidate_.anchor_s_a = candidate_a->path_s;
        candidate_.anchor_s_b = candidate_b->path_s;
        candidate_.duration = dt;
        std::ostringstream details;
        details << "pair=V" << candidate_a->id << "-V" << candidate_b->id
                << " path_gen=" << candidate_a->path_gen << "/"
                << candidate_b->path_gen << " s=" << candidate_a->path_s
                << "/" << candidate_b->path_s;
        emit("CANDIDATE", details.str(), emit_logs);
        return;
    }

    candidate_.duration += dt;
    if (candidate_.duration + 1e-9 < config_.deadlock_confirm_time) return;

    const DeadlockPairGeometry* geometry = observed_geometry;
    std::ostringstream confirmed;
    confirmed << "pair=V" << candidate_a->id << "-V" << candidate_b->id
              << " duration=" << candidate_.duration
              << " path_gen=" << candidate_a->path_gen << "/"
              << candidate_b->path_gen;
    emit("CONFIRMED", confirmed.str(), emit_logs);

    RetreatEvaluation a_retreat;
    RetreatEvaluation b_retreat;
    if (geometry != nullptr) {
        a_retreat = evaluateRetreat(*candidate_a, *candidate_b, vehicles);
        b_retreat = evaluateRetreat(*candidate_b, *candidate_a, vehicles);
    } else {
        a_retreat.retreat_vehicle_id = candidate_a->id;
        a_retreat.pass_vehicle_id = candidate_b->id;
        a_retreat.reason = "no_pair_geometry";
        b_retreat.retreat_vehicle_id = candidate_b->id;
        b_retreat.pass_vehicle_id = candidate_a->id;
        b_retreat.reason = "no_pair_geometry";
    }

    std::ostringstream evaluation;
    const int preferred_priority_id =
        geometry != nullptr ? geometry->preferred_priority_vehicle_id : -1;
    evaluation << "pair=V" << candidate_a->id << "-V" << candidate_b->id
               << " preferred_priority=V" << preferred_priority_id
               << " a_retreat_feasible=" << (a_retreat.feasible ? 1 : 0)
               << " a_distance=" << a_retreat.distance
               << " a_reason=" << a_retreat.reason
               << " b_retreat_feasible=" << (b_retreat.feasible ? 1 : 0)
               << " b_distance=" << b_retreat.distance
               << " b_reason=" << b_retreat.reason;
    emit("EVAL", evaluation.str(), emit_logs);

    const RetreatEvaluation* selected = nullptr;
    std::string selection_reason;
    const bool a_is_priority = preferred_priority_id == candidate_a->id;
    const bool b_is_priority = preferred_priority_id == candidate_b->id;
    const RetreatEvaluation* preferred_retreat = nullptr;
    const RetreatEvaluation* safety_fallback = nullptr;
    if (a_is_priority || b_is_priority) {
        preferred_retreat = a_is_priority ? &b_retreat : &a_retreat;
        safety_fallback = a_is_priority ? &a_retreat : &b_retreat;
    } else {
        // priorityWinner() can return -1 only when priority tiebreaking is
        // explicitly disabled. Keep recovery deterministic without reviving
        // distance-based role selection.
        preferred_retreat = candidate_a->id < candidate_b->id
            ? &b_retreat : &a_retreat;
        safety_fallback = preferred_retreat == &a_retreat
            ? &b_retreat : &a_retreat;
    }
    if (preferred_retreat->feasible) {
        selected = preferred_retreat;
        selection_reason = "priority_yielding_vehicle_retreat";
    } else if (safety_fallback->feasible) {
        selected = safety_fallback;
        selection_reason = "preferred_retreat_infeasible";
    }

    candidate_ = {};
    if (selected == nullptr) {
        transaction_.phase = RecoveryPhase::UNRESOLVED;
        transaction_.retreat_vehicle_id = candidate_a->id;
        transaction_.pass_vehicle_id = candidate_b->id;
        transaction_.retreat_path_gen = candidate_a->path_gen;
        transaction_.pass_path_gen = candidate_b->path_gen;
        transaction_.reason = "both_retreat_candidates_infeasible";
        refreshDirective();
        emit("UNRESOLVED", evaluation.str(), emit_logs);
        return;
    }

    const VehicleAgent* retreat = vehicleById(
        vehicles, selected->retreat_vehicle_id);
    const VehicleAgent* passer = vehicleById(
        vehicles, selected->pass_vehicle_id);
    transaction_.phase = RecoveryPhase::RETREAT;
    transaction_.retreat_vehicle_id = selected->retreat_vehicle_id;
    transaction_.pass_vehicle_id = selected->pass_vehicle_id;
    transaction_.retreat_path_gen = retreat->path_gen;
    transaction_.pass_path_gen = passer->path_gen;
    transaction_.retreat_target_s = selected->target_s;
    transaction_.pass_clear_s = selected->pass_clear_s;
    transaction_.pass_track_length = passer->track.length();
    transaction_.retreat_distance = selected->distance;
    const double recovery_speed = std::max(
        1e-6, config_.deadlock_retreat_speed);
    transaction_.estimated_retreat_time = selected->distance / recovery_speed;
    transaction_.reason = selection_reason;
    refreshDirective();
    std::ostringstream selection;
    selection << "pair=V" << candidate_a->id << "-V" << candidate_b->id
              << " retreat=V" << retreat->id << " pass=V" << passer->id
              << " target_s=" << selected->target_s
              << " pass_clear_s=" << selected->pass_clear_s
              << " distance=" << selected->distance
              << " estimated_time=" << transaction_.estimated_retreat_time
              << " selection_reason=" << selection_reason;
    emit("SELECT", selection.str(), emit_logs);
}

DeadlockManager::Snapshot DeadlockManager::snapshot() const {
    return Snapshot{candidate_, transaction_, cooldown_, directive_};
}

void DeadlockManager::restore(const Snapshot& snapshot) {
    candidate_ = snapshot.candidate;
    transaction_ = snapshot.transaction;
    cooldown_ = snapshot.cooldown;
    directive_ = snapshot.directive;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
