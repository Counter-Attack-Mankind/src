#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/cluster_admission_counterfactual_simulator.h"

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
    a.x = 0.0;
    a.y = 0.0;
    a.theta = 0.0;
    RoughWp b = a;
    b.x = 4.0;
    path.push_back(a);
    path.push_back(b);
    mv::PathTrack value;
    value.set(path);
    return value;
}

mv::FutureConflictCluster cluster() {
    mv::FutureConflictCluster value;
    value.cluster_id = 1;
    value.horizon_snapshot_id = 100;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    mv::FutureConflictZone z0;
    z0.future_zone_id = 2;
    z0.vehicle_a = 0;
    z0.vehicle_b = 1;
    z0.s_a_enter = 0.8;
    z0.s_a_exit = 3.125;
    z0.s_b_enter = 0.55;
    z0.s_b_exit = 3.05;
    z0.path_generation_a = 10;
    z0.path_generation_b = 20;
    mv::FutureConflictZone z1 = z0;
    z1.future_zone_id = 3;
    z1.s_a_enter = 2.375;
    z1.s_a_exit = 2.55;
    z1.s_b_enter = 0.5;
    z1.s_b_exit = 1.025;
    value.member_zone_ids = {2, 3};
    value.member_zones = {z0, z1};
    return value;
}

mv::ClusterReservationShadow arbitration() {
    mv::ClusterReservationShadow value;
    value.cluster_id = 1;
    value.horizon_snapshot_id = 100;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.holder_id = 1;
    value.waiter_id = 0;
    value.member_zone_ids = {2, 3};
    value.first_enter_s_a = 0.8;
    value.first_enter_s_b = 0.5;
    value.last_exit_s_a = 3.125;
    value.last_exit_s_b = 3.05;
    return value;
}

mv::ClusterAdmissionConstraint constraint() {
    mv::ClusterAdmissionConstraint value;
    value.cluster_id = 1;
    value.horizon_snapshot_id = 100;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.holder_id = 1;
    value.waiter_id = 0;
    value.member_zone_ids = {2, 3};
    value.cluster_enter_s = 0.8;
    value.cluster_exit_s = 3.125;
    value.earliest_stop_s = 0.79;
    value.required_clearance_s = 0.01;
    value.waiter_path_generation = 10;
    value.admission_feasible = true;
    value.admission_reason = "dynamic_admission_feasible";
    mv::ClusterAdmissionHolderLifecycle lifecycle;
    lifecycle.segment_id = 0;
    lifecycle.path_generation = 20;
    lifecycle.cluster_enter_s = 0.5;
    lifecycle.cluster_exit_s = 3.05;
    value.holder_lifecycle.push_back(lifecycle);
    return value;
}

mv::VehicleAgent vehicle(int id, int generation, double path_s,
                         double speed = 0.0) {
    mv::VehicleAgent value;
    value.id = id;
    value.path_gen = generation;
    value.path_s = path_s;
    value.current_speed = speed;
    value.mode = mv::VehicleMode::ACTIVE;
    value.track = track();
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
        segment.mission_leg_id.vehicle_id = id;
        segment.mission_leg_id.expected_path_gen = id == 0 ? 10 : 20;
        segment.track = track();
        trajectory.plan.segments.push_back(segment);
        output.push_back(trajectory);
    }
    return output;
}

void testWaiterStopsAndHolderContinues() {
    mv::ClusterAdmissionCounterfactualSimulator simulator;
    std::vector<mv::VehicleAgent> vehicles{
        vehicle(0, 10, 0.76, 0.1), vehicle(1, 20, 0.2, 0.1)};
    simulator.refresh({cluster()}, {constraint()}, {arbitration()},
                      trajectories(), vehicles, 10.0);
    const auto states = simulator.step(vehicles, 10.1, 0.1, 0.2);
    bool waiter_stop = false;
    bool holder_go = false;
    for (const auto& state : states) {
        waiter_stop = waiter_stop ||
            (state.vehicle_id == 0 &&
             state.shadow_action == mv::CounterfactualShadowAction::STOP);
        holder_go = holder_go ||
            (state.vehicle_id == 1 &&
             state.shadow_action == mv::CounterfactualShadowAction::GO);
    }
    require(waiter_stop, "waiter must brake before the cluster entry");
    require(holder_go, "holder must receive a counterfactual GO");
    require(!simulator.statuses().front().waiter_entered_member_zone,
            "pre-entry waiter must remain outside every member zone");
    std::cout << "PASS waiter stop and holder continue\n";
}

void testHolderClearReleasesAndResumesWaiter() {
    mv::ClusterAdmissionCounterfactualSimulator simulator;
    std::vector<mv::VehicleAgent> vehicles{
        vehicle(0, 10, 0.76, 0.0), vehicle(1, 20, 0.2, 0.1)};
    simulator.refresh({cluster()}, {constraint()}, {arbitration()},
                      trajectories(), vehicles, 10.0);
    simulator.step(vehicles, 10.1, 0.1, 0.2);
    vehicles[1].path_s = 3.06;
    const auto states = simulator.step(vehicles, 15.0, 0.1, 0.2);
    require(states.size() == 1 && states.front().vehicle_id == 0 &&
                states.front().shadow_action ==
                    mv::CounterfactualShadowAction::GO &&
                states.front().cluster_released &&
                std::abs(states.front().resume_time - 15.0) < 1e-9,
            "holder clear must atomically release and resume the waiter");
    require(!simulator.statuses().empty() &&
                !simulator.statuses().back().active &&
                std::abs(simulator.statuses().back().holder_clear_time -
                         15.0) < 1e-9,
            "completed status must retain holder clear time");
    std::cout << "PASS holder clear, cluster release, waiter resume\n";
}

void testNoAdmissionMeansNoControl() {
    mv::ClusterAdmissionCounterfactualSimulator simulator;
    auto invalid = constraint();
    invalid.admission_feasible = false;
    const std::vector<mv::VehicleAgent> vehicles{
        vehicle(0, 10, 0.76), vehicle(1, 20, 0.2)};
    simulator.refresh({cluster()}, {invalid}, {arbitration()},
                      trajectories(), vehicles, 10.0);
    require(simulator.step(vehicles, 10.1, 0.1, 0.2).empty(),
            "invalid admission must not alter the counterfactual fleet");
    std::cout << "PASS invalid admission remains observational\n";
}

void testEnteredWaiterPreventsFalseRelease() {
    mv::ClusterAdmissionCounterfactualSimulator simulator;
    std::vector<mv::VehicleAgent> vehicles{
        vehicle(0, 10, 0.81, 0.0), vehicle(1, 20, 0.2, 0.1)};
    simulator.refresh({cluster()}, {constraint()}, {arbitration()},
                      trajectories(), vehicles, 10.0);
    simulator.step(vehicles, 10.1, 0.1, 0.2);
    vehicles[1].path_s = 3.06;
    const auto states = simulator.step(vehicles, 15.0, 0.1, 0.2);
    require(std::none_of(
                states.begin(), states.end(), [](const auto& state) {
                    return state.cluster_released;
                }),
            "holder clear alone must not release after waiter entered");
    require(!simulator.statuses().empty() &&
                simulator.statuses().front().active &&
                simulator.statuses().front().waiter_entered_member_zone,
            "violated cluster must remain active until both vehicles clear");
    std::cout << "PASS all-member release closure\n";
}

void testOppositeActiveConstraintIsRejected() {
    mv::ClusterAdmissionCounterfactualSimulator simulator;
    auto opposite_constraint = constraint();
    opposite_constraint.holder_id = 0;
    opposite_constraint.waiter_id = 1;
    opposite_constraint.cluster_enter_s = 0.5;
    opposite_constraint.cluster_exit_s = 3.05;
    opposite_constraint.earliest_stop_s = 0.49;
    opposite_constraint.waiter_path_generation = 20;
    opposite_constraint.holder_lifecycle.front().path_generation = 10;
    opposite_constraint.holder_lifecycle.front().cluster_enter_s = 0.8;
    opposite_constraint.holder_lifecycle.front().cluster_exit_s = 3.125;
    auto opposite_arbitration = arbitration();
    opposite_arbitration.holder_id = 0;
    opposite_arbitration.waiter_id = 1;
    const std::vector<mv::VehicleAgent> vehicles{
        vehicle(0, 10, 0.1), vehicle(1, 20, 0.1)};

    simulator.refresh({cluster()}, {constraint()}, {arbitration()},
                      trajectories(), vehicles, 10.0);
    simulator.refresh({cluster()}, {opposite_constraint},
                      {opposite_arbitration}, trajectories(), vehicles,
                      10.1);
    const auto events = simulator.takeEvents();
    require(std::any_of(events.begin(), events.end(), [](const auto& event) {
                return event.event ==
                    "ADMISSION_NOT_FEASIBLE_CONFLICTING_ACTIVE_CONSTRAINT";
            }),
            "opposite holder constraint must be rejected while pair lock is active");
    require(simulator.statuses().size() == 1 &&
                simulator.statuses().front().holder_id == 1,
            "existing pair holder must remain unique");
    std::cout << "PASS conflicting active constraint rejection\n";
}

}  // namespace

int main() {
    testWaiterStopsAndHolderContinues();
    testHolderClearReleasesAndResumesWaiter();
    testNoAdmissionMeansNoControl();
    testEnteredWaiterPreventsFalseRelease();
    testOppositeActiveConstraintIsRejected();
    return 0;
}
