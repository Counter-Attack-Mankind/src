#include "forklift_planner/multi_vehicle/cluster_admission_counterfactual_simulator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

long long quantize(double value) {
    return static_cast<long long>(std::llround(value * 1000.0));
}

double zoneEnterFor(const FutureConflictZone& zone, int vehicle_id) {
    return zone.vehicle_a == vehicle_id ? zone.s_a_enter : zone.s_b_enter;
}

double zoneExitFor(const FutureConflictZone& zone, int vehicle_id) {
    return zone.vehicle_a == vehicle_id ? zone.s_a_exit : zone.s_b_exit;
}

int zoneGenerationFor(const FutureConflictZone& zone, int vehicle_id) {
    return zone.vehicle_a == vehicle_id
        ? zone.path_generation_a : zone.path_generation_b;
}

}  // namespace

const char* counterfactualShadowActionName(
    CounterfactualShadowAction action) {
    switch (action) {
        case CounterfactualShadowAction::NONE: return "NONE";
        case CounterfactualShadowAction::STOP: return "STOP";
        case CounterfactualShadowAction::GO: return "GO";
    }
    return "UNKNOWN";
}

ClusterAdmissionCounterfactualSimulator::Key
ClusterAdmissionCounterfactualSimulator::keyFor(
    const FutureConflictCluster& cluster,
    const ClusterAdmissionConstraint& constraint,
    const ClusterReservationShadow& arbitration) {
    return Key{
        std::min(cluster.vehicle_a, cluster.vehicle_b),
        std::max(cluster.vehicle_a, cluster.vehicle_b),
        constraint.waiter_path_generation,
        constraint.holder_lifecycle.empty()
            ? -1 : constraint.holder_lifecycle.front().path_generation,
        quantize(constraint.cluster_enter_s),
        quantize(constraint.earliest_stop_s),
        quantize(arbitration.last_exit_s_a),
        quantize(arbitration.last_exit_s_b)};
}

const VehicleAgent* ClusterAdmissionCounterfactualSimulator::vehicleById(
    const std::vector<VehicleAgent>& vehicles, int id) {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.id == id) return &vehicle;
    }
    return nullptr;
}

bool ClusterAdmissionCounterfactualSimulator::trajectoryContainsGeneration(
    const std::vector<FutureMissionTrajectory>& trajectories,
    int vehicle_id, int path_generation) {
    for (const FutureMissionTrajectory& trajectory : trajectories) {
        if (trajectory.plan.vehicle_id != vehicle_id) continue;
        for (const FutureMissionSegment& segment : trajectory.plan.segments) {
            if (segment.mission_leg_id.expected_path_gen == path_generation &&
                segment.type == FutureSegmentType::MOTION) {
                return true;
            }
        }
    }
    return false;
}

void ClusterAdmissionCounterfactualSimulator::refresh(
    const std::vector<FutureConflictCluster>& clusters,
    const std::vector<ClusterAdmissionConstraint>& constraints,
    const std::vector<ClusterReservationShadow>& arbitrations,
    const std::vector<FutureMissionTrajectory>& trajectories,
    const std::vector<VehicleAgent>& vehicles,
    double sim_time) {
    std::set<Key> refreshed;
    for (const ClusterAdmissionConstraint& constraint : constraints) {
        if (!constraint.admission_feasible ||
            constraint.holder_id < 0 || constraint.waiter_id < 0 ||
            constraint.holder_lifecycle.empty()) {
            continue;
        }
        const auto cluster_it = std::find_if(
            clusters.begin(), clusters.end(),
            [&](const FutureConflictCluster& cluster) {
                return cluster.cluster_id == constraint.cluster_id &&
                    cluster.horizon_snapshot_id ==
                        constraint.horizon_snapshot_id &&
                    cluster.vehicle_a == constraint.vehicle_a &&
                    cluster.vehicle_b == constraint.vehicle_b;
            });
        const auto arbitration_it = std::find_if(
            arbitrations.begin(), arbitrations.end(),
            [&](const ClusterReservationShadow& arbitration) {
                return arbitration.cluster_id == constraint.cluster_id &&
                    arbitration.horizon_snapshot_id ==
                        constraint.horizon_snapshot_id &&
                    arbitration.vehicle_a == constraint.vehicle_a &&
                    arbitration.vehicle_b == constraint.vehicle_b;
            });
        if (cluster_it == clusters.end() ||
            arbitration_it == arbitrations.end() ||
            cluster_it->member_zones.empty()) {
            continue;
        }

        const int holder_generation =
            constraint.holder_lifecycle.front().path_generation;
        const int waiter_generation = constraint.waiter_path_generation;
        bool holder_trajectory_valid = true;
        for (const ClusterAdmissionHolderLifecycle& lifecycle :
             constraint.holder_lifecycle) {
            holder_trajectory_valid = holder_trajectory_valid &&
                trajectoryContainsGeneration(
                    trajectories, constraint.holder_id,
                    lifecycle.path_generation);
        }
        if (!holder_trajectory_valid ||
            !trajectoryContainsGeneration(
                trajectories, constraint.waiter_id, waiter_generation)) {
            continue;
        }

        double holder_last_exit = 0.0;
        double waiter_enter = std::numeric_limits<double>::infinity();
        double waiter_last_exit = 0.0;
        bool generations_consistent = true;
        for (const FutureConflictZone& zone : cluster_it->member_zones) {
            holder_last_exit = std::max(
                holder_last_exit,
                zoneExitFor(zone, constraint.holder_id));
            waiter_enter = std::min(
                waiter_enter,
                zoneEnterFor(zone, constraint.waiter_id));
            waiter_last_exit = std::max(
                waiter_last_exit,
                zoneExitFor(zone, constraint.waiter_id));
            generations_consistent = generations_consistent &&
                zoneGenerationFor(zone, constraint.waiter_id) ==
                    waiter_generation;
        }
        if (!generations_consistent || !std::isfinite(waiter_enter)) continue;

        const Key key = keyFor(*cluster_it, constraint, *arbitration_it);
        bool opposite_active_constraint = false;
        for (const auto& active_item : controls_) {
            const CounterfactualClusterStatus& active =
                active_item.second.status;
            const bool same_pair =
                std::min(active.vehicle_a, active.vehicle_b) ==
                    std::min(constraint.vehicle_a,
                             constraint.vehicle_b) &&
                std::max(active.vehicle_a, active.vehicle_b) ==
                    std::max(constraint.vehicle_a,
                             constraint.vehicle_b);
            if (active.active && same_pair &&
                active.holder_id != constraint.holder_id) {
                opposite_active_constraint = true;
                break;
            }
        }
        if (opposite_active_constraint) {
            CounterfactualClusterStatus rejected;
            rejected.cluster_id = constraint.cluster_id;
            rejected.horizon_snapshot_id =
                constraint.horizon_snapshot_id;
            rejected.vehicle_a = constraint.vehicle_a;
            rejected.vehicle_b = constraint.vehicle_b;
            rejected.holder_id = constraint.holder_id;
            rejected.waiter_id = constraint.waiter_id;
            rejected.member_zone_ids = constraint.member_zone_ids;
            rejected.waiter_stop_s = constraint.earliest_stop_s;
            rejected.admission_braking_feasible = false;
            events_.push_back({
                "ADMISSION_NOT_FEASIBLE_CONFLICTING_ACTIVE_CONSTRAINT",
                rejected});
            continue;
        }
        refreshed.insert(key);
        auto found = controls_.find(key);
        if (found == controls_.end()) {
            Control control;
            control.status.cluster_id = constraint.cluster_id;
            control.status.horizon_snapshot_id =
                constraint.horizon_snapshot_id;
            control.status.vehicle_a = constraint.vehicle_a;
            control.status.vehicle_b = constraint.vehicle_b;
            control.status.holder_id = constraint.holder_id;
            control.status.waiter_id = constraint.waiter_id;
            control.status.member_zone_ids = constraint.member_zone_ids;
            control.status.holder_path_generation = holder_generation;
            control.status.waiter_path_generation = waiter_generation;
            control.status.holder_last_exit_s = holder_last_exit;
            control.status.waiter_enter_s = waiter_enter;
            control.status.waiter_last_exit_s = waiter_last_exit;
            control.status.waiter_stop_s = constraint.earliest_stop_s;
            control.status.created_time = sim_time;
            control.status.active = true;
            control.status.holder_lifecycle =
                constraint.holder_lifecycle;
            control.constraint = constraint;
            found = controls_.emplace(key, std::move(control)).first;
            events_.push_back({"CREATE", found->second.status});
        } else {
            found->second.status.horizon_snapshot_id =
                constraint.horizon_snapshot_id;
            found->second.status.cluster_id = constraint.cluster_id;
            found->second.status.member_zone_ids =
                constraint.member_zone_ids;
            found->second.constraint = constraint;
        }
    }

    // A control remains closed-loop active even if a later horizon no longer
    // predicts simultaneous overlap. It is released only when the admitted
    // holder clears every member or either expected path is invalidated.
    for (auto it = controls_.begin(); it != controls_.end();) {
        const VehicleAgent* holder = vehicleById(
            vehicles, it->second.status.holder_id);
        const VehicleAgent* waiter = vehicleById(
            vehicles, it->second.status.waiter_id);
        const bool invalid = holder == nullptr || waiter == nullptr;
        if (!invalid) {
            ++it;
            continue;
        }
        it->second.status.active = false;
        it->second.status.cluster_release_time = sim_time;
        it->second.status.waiter_resume_time = sim_time;
        events_.push_back({"INVALIDATE", it->second.status});
        completed_statuses_.push_back(it->second.status);
        it = controls_.erase(it);
    }
    rebuildStatusCache();
}

std::vector<ShadowVehicleState>
ClusterAdmissionCounterfactualSimulator::step(
    const std::vector<VehicleAgent>& vehicles,
    double sim_time, double dt, double max_decel) {
    std::vector<ShadowVehicleState> output;
    for (auto it = controls_.begin(); it != controls_.end();) {
        Control& control = it->second;
        CounterfactualClusterStatus& status = control.status;
        const VehicleAgent* holder = vehicleById(vehicles, status.holder_id);
        const VehicleAgent* waiter = vehicleById(vehicles, status.waiter_id);
        if (holder == nullptr || waiter == nullptr) {
            status.active = false;
            status.cluster_release_time = sim_time;
            status.waiter_resume_time = sim_time;
            events_.push_back({"INVALIDATE", status});
            completed_statuses_.push_back(status);
            it = controls_.erase(it);
            continue;
        }

        bool holder_cleared = true;
        for (const ClusterAdmissionHolderLifecycle& lifecycle :
             status.holder_lifecycle) {
            const bool lifecycle_cleared =
                holder->path_gen > lifecycle.path_generation ||
                (holder->path_gen == lifecycle.path_generation &&
                 holder->path_s > lifecycle.cluster_exit_s + 1e-9);
            holder_cleared = holder_cleared && lifecycle_cleared;
        }
        const bool waiter_clear =
            waiter->path_gen < status.waiter_path_generation ||
            waiter->path_gen > status.waiter_path_generation ||
            (waiter->path_gen == status.waiter_path_generation &&
             (waiter->path_s < status.waiter_enter_s - 1e-9 ||
              waiter->path_s > status.waiter_last_exit_s + 1e-9));
        if (holder_cleared && waiter_clear) {
            status.holder_clear_time = sim_time;
            status.cluster_release_time = sim_time;
            status.waiter_resume_time = sim_time;
            status.active = false;
            events_.push_back({"HOLDER_CLEAR", status});
            events_.push_back({"RELEASE", status});
            events_.push_back({"RESUME", status});

            ShadowVehicleState resumed;
            resumed.vehicle_id = status.waiter_id;
            resumed.cluster_id = status.cluster_id;
            resumed.holder_id = status.holder_id;
            resumed.waiter_id = status.waiter_id;
            resumed.shadow_action = CounterfactualShadowAction::GO;
            resumed.cluster_released = true;
            resumed.resume_time = sim_time;
            resumed.waiter_stop_s = status.waiter_stop_s;
            resumed.cluster_enter_s = status.waiter_enter_s;
            resumed.reason = "cluster_released_waiter_resume";
            output.push_back(resumed);

            completed_statuses_.push_back(status);
            it = controls_.erase(it);
            continue;
        }

        bool holder_on_protected_lifecycle = false;
        for (const ClusterAdmissionHolderLifecycle& lifecycle :
             status.holder_lifecycle) {
            holder_on_protected_lifecycle =
                holder_on_protected_lifecycle ||
                holder->path_gen == lifecycle.path_generation;
        }
        if (holder->active() && holder_on_protected_lifecycle) {
            ShadowVehicleState go;
            go.vehicle_id = status.holder_id;
            go.cluster_id = status.cluster_id;
            go.holder_id = status.holder_id;
            go.waiter_id = status.waiter_id;
            go.shadow_action = CounterfactualShadowAction::GO;
            go.waiter_stop_s = status.waiter_stop_s;
            go.cluster_enter_s = status.waiter_enter_s;
            go.reason = "cluster_holder_continue";
            go.baseline_action = holder->action;
            go.constrained_action = VehicleAction::NOMINAL;
            go.action_changed = holder->action != VehicleAction::NOMINAL;
            output.push_back(go);
        }

        if (waiter->active()) {
            const ClusterAdmissionDecisionShadow decision =
                evaluator_.evaluateDecision(
                    control.constraint, *waiter, dt, max_decel);
            if (decision.entered_cluster) {
                if (!status.waiter_entered_member_zone) {
                    status.waiter_entered_member_zone = true;
                    events_.push_back(
                        {"DYNAMIC_ADMISSION_BOUNDARY_VIOLATION", status});
                }
            }
            if (decision.should_stop_now) {
                if (control.waiting_started_time < 0.0) {
                    control.waiting_started_time = sim_time;
                }
                if (!control.waiter_stop_reported) {
                    control.waiter_stop_reported = true;
                    events_.push_back({"WAITER_STOP", status});
                }
                ShadowVehicleState stop;
                stop.vehicle_id = status.waiter_id;
                stop.cluster_id = status.cluster_id;
                stop.holder_id = status.holder_id;
                stop.waiter_id = status.waiter_id;
                stop.shadow_action = CounterfactualShadowAction::STOP;
                stop.cluster_waiting = true;
                stop.waiter_stop_s = status.waiter_stop_s;
                stop.cluster_enter_s = status.waiter_enter_s;
                stop.waiting_duration =
                    std::max(0.0, sim_time - control.waiting_started_time);
                stop.reason = "cluster_admission_wait";
                stop.baseline_action = decision.baseline_action;
                stop.constrained_action = decision.constrained_action;
                stop.action_changed = decision.action_changed;
                stop.distance_to_stop_s = decision.distance_to_stop_s;
                stop.required_braking_distance =
                    decision.required_braking_distance;
                output.push_back(stop);
            }
        }
        ++it;
    }
    rebuildStatusCache();
    return output;
}

std::vector<CounterfactualClusterEvent>
ClusterAdmissionCounterfactualSimulator::takeEvents() {
    std::vector<CounterfactualClusterEvent> output;
    output.swap(events_);
    return output;
}

void ClusterAdmissionCounterfactualSimulator::rebuildStatusCache() {
    status_cache_.clear();
    for (const auto& item : controls_) {
        status_cache_.push_back(item.second.status);
    }
    const std::size_t keep = std::min<std::size_t>(completed_statuses_.size(), 8);
    if (keep > 0) {
        status_cache_.insert(
            status_cache_.end(), completed_statuses_.end() - keep,
            completed_statuses_.end());
    }
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
