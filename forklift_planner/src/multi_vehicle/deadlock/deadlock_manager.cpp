#include "forklift_planner/multi_vehicle/deadlock/deadlock_manager.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

double vehiclePoseS(const VehicleAgent& vehicle) {
    return vehicle.mode == VehicleMode::DWELL
               ? vehicle.track.length()
               : vehicle.path_s;
}

template <typename Callback>
bool sampleInterval(double begin, double end, double step,
                    const Callback& callback) {
    const double direction = end >= begin ? 1.0 : -1.0;
    const double distance = std::abs(end - begin);
    const int count = std::max(1, static_cast<int>(std::ceil(distance / step)));
    for (int i = 0; i <= count; ++i) {
        const double ratio = static_cast<double>(i) / count;
        if (!callback(begin + direction * distance * ratio)) return false;
    }
    return true;
}

}  // namespace

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

RecoveryMotion RecoveryDirective::motionFor(int vehicle_id) const {
    if (phase == RecoveryPhase::RETREAT) {
        if (vehicle_id == retreat_vehicle_id) return RecoveryMotion::RETREAT;
        if (vehicle_id == pass_vehicle_id) return RecoveryMotion::HOLD;
    } else if (phase == RecoveryPhase::PASS) {
        if (vehicle_id == retreat_vehicle_id) return RecoveryMotion::HOLD;
    } else if (phase == RecoveryPhase::UNRESOLVED ||
               phase == RecoveryPhase::ABORT) {
        if (vehicle_id == retreat_vehicle_id || vehicle_id == pass_vehicle_id) {
            return RecoveryMotion::HOLD;
        }
    }
    return RecoveryMotion::NORMAL;
}

DeadlockManager::DeadlockManager(const MapParam& map_param,
                                 const MultiVehicleConfig& config)
    : map_param_(map_param), config_(config) {}

const VehicleAgent* DeadlockManager::vehicleById(
    const std::vector<VehicleAgent>& vehicles, int id) const {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.id == id) return &vehicle;
    }
    return nullptr;
}

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

bool DeadlockManager::retreatSweepClear(
    const VehicleAgent& retreat, const VehicleAgent& passer,
    const std::vector<VehicleAgent>& vehicles, double target_s) const {
    const double sweep_step = std::max(
        0.005, std::min(0.02, config_.path_validation_step));
    const double inflation = 0.5 * config_.deadlock_retreat_clearance +
                             0.5 * sweep_step;
    return sampleInterval(retreat.path_s, target_s, sweep_step,
                          [&](double retreat_s) {
        const OBB body = makeBody(retreat.track.poseAtS(retreat_s),
                                  map_param_, inflation);
        for (const VehicleAgent& other : vehicles) {
            if (other.id == retreat.id || other.track.empty() ||
                other.mode == VehicleMode::NEED_TASK) {
                continue;
            }
            const double other_s = other.id == passer.id
                                       ? passer.path_s
                                       : vehiclePoseS(other);
            const OBB obstacle = makeBody(other.track.poseAtS(other_s),
                                          map_param_, inflation);
            if (overlaps(body, obstacle)) return false;
        }
        return true;
    });
}

DeadlockManager::RetreatEvaluation DeadlockManager::evaluateRetreat(
    const VehicleAgent& retreat, const VehicleAgent& passer,
    const std::vector<VehicleAgent>& vehicles,
    const DeadlockPairGeometry& geometry) const {
    RetreatEvaluation result;
    result.retreat_vehicle_id = retreat.id;
    result.pass_vehicle_id = passer.id;
    if (retreat.track.empty() || passer.track.empty()) {
        result.reason = "empty_track";
        return result;
    }

    const bool geometry_reversed = geometry.vehicle_a != retreat.id;
    const PotentialConflictZone* selected = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const PotentialConflictZone& zone : geometry.zones) {
        const double retreat_enter = geometry_reversed
                                         ? zone.s_other_enter
                                         : zone.s_self_enter;
        const double retreat_exit = geometry_reversed
                                        ? zone.s_other_exit
                                        : zone.s_self_exit;
        const double pass_enter = geometry_reversed
                                      ? zone.s_self_enter
                                      : zone.s_other_enter;
        const double pass_exit = geometry_reversed
                                     ? zone.s_self_exit
                                     : zone.s_other_exit;
        if (retreat_exit + 1e-9 < retreat.path_s ||
            pass_exit + 1e-9 < passer.path_s) {
            continue;
        }
        const double distance = std::max(0.0, retreat_enter - retreat.path_s) +
                                std::max(0.0, pass_enter - passer.path_s);
        if (distance < best_distance) {
            best_distance = distance;
            selected = &zone;
        }
    }
    if (selected == nullptr) {
        result.reason = "no_active_conflict_corridor";
        return result;
    }

    double retreat_component_enter = geometry_reversed
        ? selected->s_other_enter : selected->s_self_enter;
    double retreat_component_exit = geometry_reversed
        ? selected->s_other_exit : selected->s_self_exit;
    double pass_component_enter = geometry_reversed
        ? selected->s_self_enter : selected->s_other_enter;
    double zone_pass_exit = geometry_reversed
        ? selected->s_self_exit : selected->s_other_exit;
    bool expanded = true;
    while (expanded) {
        expanded = false;
        for (const PotentialConflictZone& zone : geometry.zones) {
            const double retreat_enter = geometry_reversed
                ? zone.s_other_enter : zone.s_self_enter;
            const double retreat_exit = geometry_reversed
                ? zone.s_other_exit : zone.s_self_exit;
            const double pass_enter = geometry_reversed
                ? zone.s_self_enter : zone.s_other_enter;
            const double pass_exit = geometry_reversed
                ? zone.s_self_exit : zone.s_other_exit;
            const bool touches_retreat =
                retreat_enter <= retreat_component_exit +
                                      config_.deadlock_retreat_clearance &&
                retreat_exit + config_.deadlock_retreat_clearance >=
                    retreat_component_enter;
            const bool touches_pass =
                pass_enter <= zone_pass_exit +
                                  config_.deadlock_retreat_clearance &&
                pass_exit + config_.deadlock_retreat_clearance >=
                    pass_component_enter;
            if (!touches_retreat || !touches_pass) continue;
            const double old_retreat_enter = retreat_component_enter;
            const double old_retreat_exit = retreat_component_exit;
            const double old_pass_enter = pass_component_enter;
            const double old_pass_exit = zone_pass_exit;
            retreat_component_enter = std::min(retreat_component_enter,
                                               retreat_enter);
            retreat_component_exit = std::max(retreat_component_exit,
                                              retreat_exit);
            pass_component_enter = std::min(pass_component_enter, pass_enter);
            zone_pass_exit = std::max(zone_pass_exit, pass_exit);
            expanded = old_retreat_enter != retreat_component_enter ||
                       old_retreat_exit != retreat_component_exit ||
                       old_pass_enter != pass_component_enter ||
                       old_pass_exit != zone_pass_exit;
        }
    }
    const double sweep_step = std::max(
        0.005, std::min(0.02, config_.path_validation_step));
    result.pass_clear_s = std::min(
        passer.track.length(), zone_pass_exit +
                                   config_.deadlock_retreat_clearance +
                                   sweep_step);

    const double inflation = 0.5 * config_.deadlock_retreat_clearance +
                             0.5 * sweep_step;
    std::vector<OBB> pass_corridor;
    sampleInterval(passer.path_s, result.pass_clear_s, sweep_step,
                   [&](double pass_s) {
        pass_corridor.push_back(makeBody(passer.track.poseAtS(pass_s),
                                         map_param_, inflation));
        return true;
    });

    const double search_step = config_.deadlock_retreat_search_step;
    auto targetClearsCorridor = [&](double candidate_s) {
        const OBB stopped = makeBody(retreat.track.poseAtS(candidate_s),
                                     map_param_, inflation);
        for (const OBB& pass_body : pass_corridor) {
            if (overlaps(stopped, pass_body)) return false;
        }
        return true;
    };

    bool found = false;
    for (double candidate_s = std::max(0.0, retreat.path_s - search_step);;
         candidate_s = std::max(0.0, candidate_s - search_step)) {
        if (targetClearsCorridor(candidate_s) &&
            retreatSweepClear(retreat, passer, vehicles, candidate_s)) {
            result.target_s = candidate_s;
            found = true;
            break;
        }
        if (candidate_s <= 1e-9) break;
    }
    if (!found) {
        result.reason = "retreat_sweep_or_corridor_blocked";
        return result;
    }

    result.feasible = true;
    result.distance = retreat.path_s - result.target_s;
    result.reason = "clear";
    return result;
}

void DeadlockManager::refreshDirective() {
    directive_.phase = transaction_.phase;
    directive_.retreat_vehicle_id = transaction_.retreat_vehicle_id;
    directive_.pass_vehicle_id = transaction_.pass_vehicle_id;
    directive_.retreat_path_gen = transaction_.retreat_path_gen;
    directive_.pass_path_gen = transaction_.pass_path_gen;
    directive_.retreat_target_s = transaction_.retreat_target_s;
    directive_.pass_clear_s = transaction_.pass_clear_s;
    directive_.retreat_distance = transaction_.retreat_distance;
    directive_.estimated_retreat_time = transaction_.estimated_retreat_time;
    directive_.reason = transaction_.reason;
}

void DeadlockManager::emit(const char* event, const std::string& details,
                           bool enabled) const {
    if (!enabled || !log_sink_) return;
    log_sink_(std::string("[DEADLOCK] event=") + event + " " + details);
}

void DeadlockManager::abort(const std::string& reason, bool emit_logs) {
    transaction_.phase = RecoveryPhase::ABORT;
    transaction_.reason = reason;
    refreshDirective();
    emit("ABORT", "pair=V" + std::to_string(transaction_.retreat_vehicle_id) +
                      "-V" + std::to_string(transaction_.pass_vehicle_id) +
                      " reason=" + reason,
         emit_logs);
}

void DeadlockManager::update(
    const std::vector<VehicleAgent>& vehicles,
    const std::vector<DeadlockPairGeometry>& pair_geometry,
    double dt, bool emit_logs) {
    if (!config_.deadlock_enabled) {
        candidate_ = {};
        transaction_ = {};
        directive_ = {};
        return;
    }

    if (transaction_.phase != RecoveryPhase::NONE) {
        if (transaction_.phase == RecoveryPhase::CLEAR) {
            transaction_ = {};
            refreshDirective();
            return;
        }
        const VehicleAgent* retreat = vehicleById(
            vehicles, transaction_.retreat_vehicle_id);
        const VehicleAgent* passer = vehicleById(
            vehicles, transaction_.pass_vehicle_id);
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
                transaction_.phase = RecoveryPhase::PASS;
                transaction_.reason = "retreat_target_reached";
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

        if (transaction_.phase == RecoveryPhase::PASS) {
            if (passer->path_s >= transaction_.pass_clear_s - 1e-9) {
                std::ostringstream details;
                details << "pair=V" << retreat->id << "-V" << passer->id
                        << " pass_s=" << passer->path_s
                        << " pass_clear_s=" << transaction_.pass_clear_s;
                transaction_.phase = RecoveryPhase::CLEAR;
                transaction_.reason = "passer_cleared_corridor";
                refreshDirective();
                emit("CLEAR", details.str(), emit_logs);
            }
            return;
        }
        return;
    }

    const VehicleAgent* candidate_a = nullptr;
    const VehicleAgent* candidate_b = nullptr;
    for (const VehicleAgent& a : vehicles) {
        if (a.mode != VehicleMode::ACTIVE ||
            a.action != VehicleAction::STOP || a.blocker_id < 0) {
            continue;
        }
        const VehicleAgent* b = vehicleById(vehicles, a.blocker_id);
        if (b == nullptr || b->mode != VehicleMode::ACTIVE ||
            b->action != VehicleAction::STOP || b->blocker_id != a.id) {
            continue;
        }
        if (a.id < b->id) {
            candidate_a = &a;
            candidate_b = b;
            break;
        }
    }

    if (candidate_a == nullptr) {
        candidate_ = {};
        directive_ = {};
        return;
    }

    const DeadlockPairGeometry* observed_geometry = geometryFor(
        pair_geometry, candidate_a->id, candidate_b->id);
    if (observed_geometry == nullptr || observed_geometry->zones.empty()) {
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
    if (geometry != nullptr && !geometry->zones.empty()) {
        a_retreat = evaluateRetreat(*candidate_a, *candidate_b, vehicles,
                                    *geometry);
        b_retreat = evaluateRetreat(*candidate_b, *candidate_a, vehicles,
                                    *geometry);
    } else {
        a_retreat.retreat_vehicle_id = candidate_a->id;
        a_retreat.pass_vehicle_id = candidate_b->id;
        a_retreat.reason = "no_pair_geometry";
        b_retreat.retreat_vehicle_id = candidate_b->id;
        b_retreat.pass_vehicle_id = candidate_a->id;
        b_retreat.reason = "no_pair_geometry";
    }

    std::ostringstream evaluation;
    evaluation << "pair=V" << candidate_a->id << "-V" << candidate_b->id
               << " a_retreat_feasible=" << (a_retreat.feasible ? 1 : 0)
               << " a_distance=" << a_retreat.distance
               << " a_reason=" << a_retreat.reason
               << " b_retreat_feasible=" << (b_retreat.feasible ? 1 : 0)
               << " b_distance=" << b_retreat.distance
               << " b_reason=" << b_retreat.reason;
    emit("EVAL", evaluation.str(), emit_logs);

    const RetreatEvaluation* selected = nullptr;
    if (a_retreat.feasible && b_retreat.feasible) {
        if (std::abs(a_retreat.distance - b_retreat.distance) <= 1e-9) {
            selected = a_retreat.retreat_vehicle_id < b_retreat.retreat_vehicle_id
                           ? &a_retreat : &b_retreat;
        } else {
            selected = a_retreat.distance < b_retreat.distance
                           ? &a_retreat : &b_retreat;
        }
    } else if (a_retreat.feasible) {
        selected = &a_retreat;
    } else if (b_retreat.feasible) {
        selected = &b_retreat;
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
    transaction_.retreat_distance = selected->distance;
    const double recovery_speed = std::max(
        1e-6, config_.nominal_speed * config_.creep_ratio);
    transaction_.estimated_retreat_time = selected->distance / recovery_speed;
    transaction_.reason = "minimum_safe_retreat";
    refreshDirective();
    std::ostringstream selection;
    selection << "pair=V" << candidate_a->id << "-V" << candidate_b->id
              << " retreat=V" << retreat->id << " pass=V" << passer->id
              << " target_s=" << selected->target_s
              << " pass_clear_s=" << selected->pass_clear_s
              << " distance=" << selected->distance
              << " estimated_time=" << transaction_.estimated_retreat_time;
    emit("SELECT", selection.str(), emit_logs);
}

bool DeadlockManager::passOverride(int vehicle_a, int vehicle_b) const {
    if (transaction_.phase != RecoveryPhase::PASS) return false;
    return (transaction_.retreat_vehicle_id == vehicle_a &&
            transaction_.pass_vehicle_id == vehicle_b) ||
           (transaction_.retreat_vehicle_id == vehicle_b &&
            transaction_.pass_vehicle_id == vehicle_a);
}

DeadlockManager::Snapshot DeadlockManager::snapshot() const {
    return Snapshot{candidate_, transaction_, directive_};
}

void DeadlockManager::restore(const Snapshot& snapshot) {
    candidate_ = snapshot.candidate;
    transaction_ = snapshot.transaction;
    directive_ = snapshot.directive;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
