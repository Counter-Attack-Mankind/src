#include "forklift_planner/multi_vehicle/dynamic_cluster_admission_shadow.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

const VehicleAgent& vehicleForId(const VehicleAgent& a,
                                 const VehicleAgent& b, int id) {
    return a.id == id ? a : b;
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

int zoneSegmentFor(const FutureConflictZone& zone, int vehicle_id) {
    return zone.vehicle_a == vehicle_id
        ? zone.segment_id_a : zone.segment_id_b;
}

MissionPhase zonePhaseFor(const FutureConflictZone& zone, int vehicle_id) {
    return zone.vehicle_a == vehicle_id ? zone.phase_a : zone.phase_b;
}

FutureCertainty zoneCertaintyFor(const FutureConflictZone& zone,
                                 int vehicle_id) {
    return zone.vehicle_a == vehicle_id
        ? zone.certainty_a : zone.certainty_b;
}

bool trajectoryHasLifecycle(
    const std::vector<FutureMissionTrajectory>& trajectories,
    int vehicle_id, int segment_id, int generation) {
    for (const FutureMissionTrajectory& trajectory : trajectories) {
        if (trajectory.plan.vehicle_id != vehicle_id) continue;
        for (const FutureMissionSegment& segment : trajectory.plan.segments) {
            if (segment.segment_id == segment_id &&
                segment.mission_leg_id.expected_path_gen == generation &&
                segment.type == FutureSegmentType::MOTION) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace

double ClusterAdmissionEvaluator::stoppingDistance(
    double speed, double dt, double max_decel) {
    const double nonnegative_speed = std::max(0.0, speed);
    return nonnegative_speed * nonnegative_speed /
               (2.0 * std::max(1e-6, max_decel)) +
           nonnegative_speed * std::max(0.0, dt);
}

ClusterAdmissionConstraint ClusterAdmissionEvaluator::buildConstraint(
    const FutureConflictCluster& cluster,
    const ClusterAdmissionShadow& admission,
    const ClusterReservationShadow& arbitration,
    const std::vector<FutureMissionTrajectory>& trajectories,
    const VehicleAgent& vehicle_a,
    const VehicleAgent& vehicle_b,
    double required_clearance_s,
    double dt,
    double max_accel,
    double max_decel,
    double waiter_curvature_speed_limit) const {
    ClusterAdmissionConstraint result;
    result.cluster_id = cluster.cluster_id;
    result.horizon_snapshot_id = cluster.horizon_snapshot_id;
    result.vehicle_a = cluster.vehicle_a;
    result.vehicle_b = cluster.vehicle_b;
    result.holder_id = admission.holder_id;
    result.waiter_id = admission.waiter_id;
    result.member_zone_ids = cluster.member_zone_ids;
    result.required_clearance_s = std::max(0.0, required_clearance_s);
    result.dt = std::max(0.0, dt);
    result.max_accel = std::max(0.0, max_accel);
    result.max_decel = std::max(0.0, max_decel);

    // A single zone is already the native arbitration unit of the legacy
    // ConflictReservation path. Dynamic cluster admission is intentionally
    // limited to the multi-zone closure gap; duplicating a single-zone lock
    // can outlive the legacy reservation and create an unnecessary wait edge.
    if (cluster.member_zones.size() < 2) {
        result.admission_reason =
            "cluster_admission_not_required_single_zone";
        return result;
    }

    const bool valid_pair =
        result.holder_id >= 0 && result.waiter_id >= 0 &&
        result.holder_id != result.waiter_id &&
        (result.holder_id == vehicle_a.id ||
         result.holder_id == vehicle_b.id) &&
        (result.waiter_id == vehicle_a.id ||
         result.waiter_id == vehicle_b.id);
    if (!valid_pair) {
        result.admission_reason = "invalid_cluster_arbitration";
        return result;
    }

    const VehicleAgent& waiter = vehicleForId(
        vehicle_a, vehicle_b, result.waiter_id);
    result.evaluated_path_s = waiter.path_s;
    result.evaluated_speed = std::max(0.0, waiter.current_speed);
    result.curvature_speed_limit =
        std::max(0.0, waiter_curvature_speed_limit);
    const double accelerated_speed =
        result.evaluated_speed + result.max_accel * result.dt;
    const double curvature_limited_next =
        result.curvature_speed_limit > 1e-9
            ? std::min(accelerated_speed, result.curvature_speed_limit)
            : accelerated_speed;
    // A curvature limit cannot instantaneously remove existing kinetic
    // energy, so never use it below the measured current speed.
    result.approach_speed_upper_bound = std::max(
        result.evaluated_speed, curvature_limited_next);
    result.required_braking_distance = stoppingDistance(
        result.approach_speed_upper_bound, result.dt, result.max_decel);

    double waiter_enter = std::numeric_limits<double>::infinity();
    double waiter_exit = 0.0;
    int waiter_generation = -1;
    bool waiter_generation_consistent = true;
    std::map<std::pair<int, int>, ClusterAdmissionHolderLifecycle>
        lifecycle_by_segment;
    for (const FutureConflictZone& zone : cluster.member_zones) {
        const int generation = zoneGenerationFor(zone, result.waiter_id);
        waiter_enter = std::min(
            waiter_enter, zoneEnterFor(zone, result.waiter_id));
        waiter_exit = std::max(
            waiter_exit, zoneExitFor(zone, result.waiter_id));
        if (waiter_generation < 0) waiter_generation = generation;
        if (waiter_generation != generation) {
            waiter_generation_consistent = false;
        }

        const int holder_generation =
            zoneGenerationFor(zone, result.holder_id);
        const int holder_segment = zoneSegmentFor(zone, result.holder_id);
        const std::pair<int, int> lifecycle_key{
            holder_generation, holder_segment};
        auto found = lifecycle_by_segment.find(lifecycle_key);
        if (found == lifecycle_by_segment.end()) {
            ClusterAdmissionHolderLifecycle lifecycle;
            lifecycle.segment_id = holder_segment;
            lifecycle.phase = zonePhaseFor(zone, result.holder_id);
            lifecycle.certainty = zoneCertaintyFor(
                zone, result.holder_id);
            lifecycle.path_generation = holder_generation;
            lifecycle.cluster_enter_s =
                zoneEnterFor(zone, result.holder_id);
            lifecycle.cluster_exit_s =
                zoneExitFor(zone, result.holder_id);
            lifecycle_by_segment.emplace(lifecycle_key, lifecycle);
        } else {
            found->second.cluster_enter_s = std::min(
                found->second.cluster_enter_s,
                zoneEnterFor(zone, result.holder_id));
            found->second.cluster_exit_s = std::max(
                found->second.cluster_exit_s,
                zoneExitFor(zone, result.holder_id));
        }
    }
    for (const auto& item : lifecycle_by_segment) {
        result.holder_lifecycle.push_back(item.second);
    }

    if (!std::isfinite(waiter_enter) || !waiter_generation_consistent ||
        waiter_generation < 0) {
        result.admission_reason = "ambiguous_waiter_cluster_entry";
        return result;
    }
    result.waiter_path_generation = waiter_generation;
    result.cluster_enter_s = waiter_enter;
    result.cluster_exit_s = waiter_exit;
    result.earliest_stop_s = std::max(
        0.0, waiter_enter - result.required_clearance_s);
    result.available_braking_distance =
        result.earliest_stop_s - waiter.path_s;
    result.waiter_already_inside =
        waiter.path_gen == waiter_generation &&
        waiter.path_s >= waiter_enter - 1e-9 &&
        waiter.path_s <= waiter_exit + 1e-9;

    if (!admission.shadow_lock_active || !admission.admission_valid) {
        result.admission_reason = "shadow_admission_not_active";
        return result;
    }
    if (arbitration.zone_level_mixed_holders) {
        // Admission is preventive. Once member zones already report
        // different holders, adding a new STOP edge can close the very wait
        // cycle being diagnosed. Leave that late state to the existing
        // arbitration and report it as dynamically inadmissible.
        result.admission_reason =
            "admission_not_feasible_mixed_holder_already_present";
        return result;
    }
    if (arbitration.all_members_cleared) {
        result.admission_reason = "cluster_already_released";
        return result;
    }
    if (waiter.path_gen != waiter_generation) {
        result.admission_reason = "waiter_path_generation_mismatch";
        return result;
    }
    if (result.waiter_already_inside) {
        result.admission_reason = "admission_not_feasible_already_inside";
        return result;
    }
    if (result.available_braking_distance < -1e-9) {
        result.admission_reason = "admission_not_feasible_stop_line_passed";
        return result;
    }
    if (result.available_braking_distance + 1e-9 <
        result.required_braking_distance) {
        result.admission_reason =
            "admission_not_feasible_insufficient_braking_distance";
        return result;
    }
    for (const ClusterAdmissionHolderLifecycle& lifecycle :
         result.holder_lifecycle) {
        if (!trajectoryHasLifecycle(
                trajectories, result.holder_id, lifecycle.segment_id,
                lifecycle.path_generation)) {
            result.admission_reason = "holder_lifecycle_not_in_trajectory";
            return result;
        }
    }

    result.admission_feasible = true;
    result.admission_reason = "dynamic_admission_feasible";
    return result;
}

ClusterAdmissionDecisionShadow
ClusterAdmissionEvaluator::evaluateDecision(
    const ClusterAdmissionConstraint& constraint,
    const VehicleAgent& waiter,
    double dt,
    double max_decel) const {
    ClusterAdmissionDecisionShadow result;
    result.cluster_id = constraint.cluster_id;
    result.holder_id = constraint.holder_id;
    result.waiter_id = constraint.waiter_id;
    result.baseline_action = waiter.action;
    result.constrained_action = waiter.action;
    result.baseline_reason = waiter.reason;
    result.stop_s = constraint.earliest_stop_s;
    result.distance_to_stop_s = result.stop_s - waiter.path_s;
    result.required_braking_distance = stoppingDistance(
        waiter.current_speed, dt, max_decel);
    result.constraint_active = constraint.admission_feasible &&
        waiter.id == constraint.waiter_id &&
        waiter.path_gen == constraint.waiter_path_generation;
    result.entered_cluster = result.constraint_active &&
        waiter.path_s >= constraint.cluster_enter_s - 1e-9 &&
        waiter.path_s <= constraint.cluster_exit_s + 1e-9;
    result.stop_boundary_passed = result.constraint_active &&
        waiter.path_s > constraint.earliest_stop_s + 1e-9;

    if (!result.constraint_active) {
        result.decision_reason = constraint.admission_reason;
        return result;
    }
    if (result.entered_cluster) {
        result.decision_reason = "dynamic_admission_boundary_violation";
        return result;
    }
    if (result.stop_boundary_passed) {
        // The discrete vehicle integrator may stop a few millimetres beyond
        // the requested stop line while still remaining outside the actual
        // cluster entry. Keep the monotone STOP constraint active when the
        // remaining distance to the resource is still dynamically feasible;
        // otherwise report infeasibility and do not force an impossible stop.
        const double distance_to_cluster =
            constraint.cluster_enter_s - waiter.path_s;
        if (distance_to_cluster + 1e-9 <
            result.required_braking_distance) {
            result.decision_reason =
                "admission_not_feasible_after_stop_boundary";
            return result;
        }
        result.should_stop_now = true;
        result.constrained_action = VehicleAction::STOP;
        result.action_changed = waiter.action != VehicleAction::STOP;
        result.decision_reason = "dynamic_cluster_hold_before_entry";
        return result;
    }
    if (result.distance_to_stop_s <=
        result.required_braking_distance + 1e-9) {
        result.should_stop_now = true;
        result.constrained_action = VehicleAction::STOP;
        result.action_changed = waiter.action != VehicleAction::STOP;
        result.decision_reason = "dynamic_cluster_brake_before";
    } else {
        result.decision_reason = "dynamic_cluster_approach";
    }
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
