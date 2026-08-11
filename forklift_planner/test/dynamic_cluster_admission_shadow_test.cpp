#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/dynamic_cluster_admission_shadow.h"

namespace mv = forklift_planner::multi_vehicle;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

mv::PathTrack track() {
    RoughPath path;
    RoughWp a;
    RoughWp b;
    b.x = 4.0;
    path.push_back(a);
    path.push_back(b);
    mv::PathTrack value;
    value.set(path);
    return value;
}

mv::VehicleAgent vehicle(int id, int generation, double path_s,
                         double speed) {
    mv::VehicleAgent value;
    value.id = id;
    value.path_gen = generation;
    value.path_s = path_s;
    value.current_speed = speed;
    value.mode = mv::VehicleMode::ACTIVE;
    value.action = mv::VehicleAction::NOMINAL;
    value.reason = "clear";
    value.track = track();
    return value;
}

mv::FutureConflictCluster cluster() {
    mv::FutureConflictCluster value;
    value.cluster_id = 4;
    value.horizon_snapshot_id = 2026;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    mv::FutureConflictZone zone;
    zone.future_zone_id = 2;
    zone.vehicle_a = 0;
    zone.vehicle_b = 1;
    zone.segment_id_a = 0;
    zone.segment_id_b = 0;
    zone.phase_a = mv::MissionPhase::TO_A1;
    zone.phase_b = mv::MissionPhase::TO_B;
    zone.certainty_a = mv::FutureCertainty::COMMITTED;
    zone.certainty_b = mv::FutureCertainty::COMMITTED;
    zone.s_a_enter = 0.8;
    zone.s_a_exit = 3.125;
    zone.s_b_enter = 0.5;
    zone.s_b_exit = 3.05;
    zone.path_generation_a = 10;
    zone.path_generation_b = 20;
    mv::FutureConflictZone connected = zone;
    connected.future_zone_id = 3;
    connected.s_a_enter = 2.0;
    connected.s_a_exit = 2.4;
    connected.s_b_enter = 2.2;
    connected.s_b_exit = 2.7;
    value.member_zone_ids = {2, 3};
    value.member_zones = {zone, connected};
    return value;
}

mv::ClusterAdmissionShadow admission() {
    mv::ClusterAdmissionShadow value;
    value.cluster_id = 4;
    value.horizon_snapshot_id = 2026;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.holder_id = 1;
    value.waiter_id = 0;
    value.member_zone_ids = {2};
    value.cluster_enter_s_a = 0.8;
    value.cluster_enter_s_b = 0.5;
    value.entry_path_generation_a = 10;
    value.entry_path_generation_b = 20;
    value.admission_valid = true;
    value.shadow_lock_active = true;
    return value;
}

mv::ClusterReservationShadow arbitration() {
    mv::ClusterReservationShadow value;
    value.cluster_id = 4;
    value.horizon_snapshot_id = 2026;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.holder_id = 1;
    value.waiter_id = 0;
    value.member_zone_ids = {2};
    value.first_enter_s_a = 0.8;
    value.first_enter_s_b = 0.5;
    value.last_exit_s_a = 3.125;
    value.last_exit_s_b = 3.05;
    return value;
}

std::vector<mv::FutureMissionTrajectory> trajectories() {
    std::vector<mv::FutureMissionTrajectory> output;
    for (int id = 0; id < 2; ++id) {
        mv::FutureMissionTrajectory trajectory;
        trajectory.plan.vehicle_id = id;
        mv::FutureMissionSegment segment;
        segment.segment_id = 0;
        segment.type = mv::FutureSegmentType::MOTION;
        segment.phase = id == 0 ? mv::MissionPhase::TO_A1
                                : mv::MissionPhase::TO_B;
        segment.mission_leg_id.vehicle_id = id;
        segment.mission_leg_id.expected_path_gen = id == 0 ? 10 : 20;
        segment.track = track();
        trajectory.plan.segments.push_back(segment);
        output.push_back(trajectory);
    }
    return output;
}

mv::ClusterAdmissionConstraint build(double waiter_s, double waiter_speed,
                                     double curvature_limit = 0.12) {
    const auto a = vehicle(0, 10, waiter_s, waiter_speed);
    const auto b = vehicle(1, 20, 0.1, 0.1);
    return mv::ClusterAdmissionEvaluator().buildConstraint(
        cluster(), admission(), arbitration(), trajectories(), a, b,
        0.01, 0.1, 0.2, 0.2, curvature_limit);
}

void testFeasibleConstraintUsesDynamicsAndCurvature() {
    const auto result = build(0.1, 0.1);
    require(result.admission_feasible,
            "far upstream waiter must produce a feasible constraint");
    require(std::abs(result.earliest_stop_s - 0.79) < 1e-9,
            "constraint must retain the geometric clearance boundary");
    require(std::abs(result.approach_speed_upper_bound - 0.12) < 1e-9,
            "next approach speed must respect acceleration and curvature");
    require(result.available_braking_distance >
                result.required_braking_distance,
            "feasible admission must have enough braking distance");
    require(result.holder_lifecycle.size() == 1 &&
                result.holder_lifecycle.front().phase ==
                    mv::MissionPhase::TO_B,
            "constraint must carry the holder lifecycle");
    std::cout << "PASS dynamically feasible constraint\n";
}

void testLateConstraintIsRejected() {
    const auto result = build(0.77, 0.1);
    require(!result.admission_feasible,
            "late geometric admission must not be enforced");
    require(result.admission_reason ==
                "admission_not_feasible_insufficient_braking_distance",
            "late admission must expose braking-distance reason");
    std::cout << "PASS late admission rejection\n";
}

void testRuntimeDecisionMatchesBrakeBeforeTrigger() {
    const auto constraint = build(0.1, 0.1);
    auto waiter = vehicle(0, 10, 0.755, 0.1);
    const auto result = mv::ClusterAdmissionEvaluator().evaluateDecision(
        constraint, waiter, 0.1, 0.2);
    require(result.should_stop_now,
            "constraint must request STOP at the existing brake threshold");
    require(result.constrained_action == mv::VehicleAction::STOP &&
                result.action_changed,
            "shadow comparison must expose NOMINAL to STOP change");
    require(std::abs(result.required_braking_distance - 0.035) < 1e-9,
            "runtime trigger must use v^2/(2a)+v*dt");
    std::cout << "PASS brakeBefore-equivalent runtime decision\n";
}

void testDiscreteOvershootIsHeldBeforeClusterEntry() {
    auto constraint = build(0.1, 0.1);
    constraint.earliest_stop_s = 0.79;
    constraint.cluster_enter_s = 0.8;
    auto waiter = vehicle(0, 10, 0.792, 0.0);
    const auto result = mv::ClusterAdmissionEvaluator().evaluateDecision(
        constraint, waiter, 0.1, 0.2);
    require(result.stop_boundary_passed && !result.entered_cluster,
            "fixture must be beyond stop line but outside cluster");
    require(result.should_stop_now &&
                result.constrained_action == mv::VehicleAction::STOP,
            "safe discrete overshoot must remain stopped before entry");
    require(result.decision_reason ==
                "dynamic_cluster_hold_before_entry",
            "discrete overshoot must be distinguished from cluster entry");
    std::cout << "PASS discrete stop-line overshoot hold\n";
}

void testExistingMixedHolderIsRejectedAsTooLate() {
    const auto a = vehicle(0, 10, 0.1, 0.1);
    const auto b = vehicle(1, 20, 0.1, 0.1);
    auto mixed = arbitration();
    mixed.zone_level_mixed_holders = true;
    const auto result = mv::ClusterAdmissionEvaluator().buildConstraint(
        cluster(), admission(), mixed, trajectories(), a, b,
        0.01, 0.1, 0.2, 0.2, 0.12);
    require(!result.admission_feasible &&
                result.admission_reason ==
                    "admission_not_feasible_mixed_holder_already_present",
            "admission must not add a STOP after mixed holders exist");
    std::cout << "PASS late mixed-holder admission rejection\n";
}

}  // namespace

int main() {
    testFeasibleConstraintUsesDynamicsAndCurvature();
    testLateConstraintIsRejected();
    testRuntimeDecisionMatchesBrakeBeforeTrigger();
    testDiscreteOvershootIsHeldBeforeClusterEntry();
    testExistingMixedHolderIsRejectedAsTooLate();
    return 0;
}
