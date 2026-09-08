#include <cmath>
#include <iostream>
#include <vector>

#include "forklift_planner/multi_vehicle/deadlock/deadlock_manager.h"

namespace {
int fail(const char* message) {
    std::cerr << "deadlock_manager_test: " << message << "\n";
    return 1;
}
}  // namespace

int main() {
    using namespace forklift_planner::multi_vehicle;
    MapParam map;
    MultiVehicleConfig config;
    config.deadlock_confirm_time = 0.2;
    config.deadlock_retreat_search_step = 0.05;
    config.deadlock_retreat_clearance = 0.01;
    config.deadlock_retreat_speed = 0.10;

    VehicleAgent a;
    a.id = 0; a.mode = VehicleMode::ACTIVE;
    a.action = VehicleAction::STOP; a.requested_action = VehicleAction::STOP;
    a.blocker_id = 1; a.path_gen = 3; a.path_s = 0.60;
    a.track.set(RoughPath{
        RoughWp{-1.0, 0.0, 0.0, WpType::FORWARD},
        RoughWp{1.0, 0.0, 0.0, WpType::FORWARD}});

    VehicleAgent b;
    b.id = 1; b.mode = VehicleMode::ACTIVE;
    b.action = VehicleAction::STOP; b.requested_action = VehicleAction::STOP;
    b.blocker_id = 0; b.path_gen = 7; b.path_s = 0.60;
    b.track.set(RoughPath{
        RoughWp{0.0, -1.0, M_PI_2, WpType::FORWARD},
        RoughWp{0.0, 1.0, M_PI_2, WpType::FORWARD}});

    DeadlockPairGeometry geometry;
    geometry.vehicle_a = a.id; geometry.vehicle_b = b.id;
    geometry.path_gen_a = a.path_gen; geometry.path_gen_b = b.path_gen;
    geometry.preferred_priority_vehicle_id = b.id;

    std::vector<VehicleAgent> vehicles{a, b};
    DeadlockManager manager(map, config);
    const auto clean = manager.snapshot();
    manager.update(vehicles, {geometry}, 0.1, false);
    manager.restore(clean);
    manager.update(vehicles, {geometry}, 0.1, false);
    if (manager.directive().phase != RecoveryPhase::NONE) {
        return fail("restored rollout time leaked into live confirmation");
    }

    manager.update(vehicles, {geometry}, 0.1, false);
    const RecoveryDirective selected = manager.directive();
    if (selected.phase != RecoveryPhase::RETREAT ||
        selected.retreat_vehicle_id != 0 || selected.pass_vehicle_id != 1 ||
        selected.retreat_target_s >= vehicles[0].path_s) {
        return fail("deterministic minimum retreat was not selected");
    }
    if (std::abs(selected.estimated_retreat_time -
                 selected.retreat_distance / config.deadlock_retreat_speed) >
        1e-9) {
        return fail("retreat estimate did not use dedicated recovery speed");
    }
    vehicles[0].path_s = selected.retreat_target_s;
    manager.update(vehicles, {geometry}, 0.1, false);
    if (manager.directive().phase != RecoveryPhase::PASS ||
        manager.directive().motionFor(0) != RecoveryMotion::HOLD ||
        manager.directive().motionFor(1) != RecoveryMotion::NORMAL) {
        return fail("PASS did not hold retreat and release passer");
    }
    manager.update(vehicles, {geometry}, 0.1, false);
    if (manager.directive().phase != RecoveryPhase::PASS ||
        manager.directive().cooldownActive()) {
        return fail("PASS cleared before passer reached pass_clear_s");
    }
    vehicles[1].path_s = selected.pass_clear_s;
    manager.update(vehicles, {geometry}, 0.1, false);
    if (manager.directive().phase != RecoveryPhase::NONE ||
        !manager.directive().cooldownActive() ||
        manager.directive().cooldown_vehicle_id != 0 ||
        manager.directive().motionFor(0) != RecoveryMotion::HOLD ||
        manager.directive().motionFor(1) != RecoveryMotion::NORMAL) {
        return fail("PASS did not clear into retreat-only cooldown");
    }

    VehicleAgent late_owner = b;
    late_owner.id = 1;
    late_owner.path_gen = 20;
    late_owner.mission_phase = MissionPhase::TO_A1;
    late_owner.path_s = 0.2;
    late_owner.track.set(RoughPath{
        RoughWp{-1.0, 10.0, 0.0, WpType::FORWARD},
        RoughWp{1.0, 10.0, 0.0, WpType::FORWARD}});
    VehicleAgent intruder = a;
    intruder.id = 0;
    intruder.path_gen = 30;
    intruder.mission_phase = MissionPhase::TO_A1;
    intruder.path_s = 0.6;
    intruder.track.set(RoughPath{
        RoughWp{-1.0, 0.0, 0.0, WpType::FORWARD},
        RoughWp{1.0, 0.0, 0.0, WpType::FORWARD}});
    A1LateOwnerRecoveryRequest late;
    late.owner_id = late_owner.id;
    late.transaction_owner_path_gen = late_owner.path_gen;
    late.owner_departure_path_gen = late_owner.path_gen + 1;
    late.intruder_id = intruder.id;
    late.intruder_path_gen = intruder.path_gen;
    late.waiter_stop_s = 0.5;
    late.frozen_owner_track = late_owner.track;
    late.frozen_intruder_track = intruder.track;
    PotentialConflictZone late_zone;
    late_zone.s_self_enter = 0.8;
    late_zone.s_self_exit = 1.2;
    late_zone.s_other_enter = 0.5;
    late_zone.s_other_exit = 0.9;
    late.closure_zones.push_back(late_zone);
    std::vector<VehicleAgent> late_vehicles{intruder, late_owner};
    DeadlockManager late_manager(map, config);
    late_manager.requestA1LateOwnerRecovery(late, late_vehicles, false);
    const RecoveryDirective late_selected = late_manager.directive();
    if (late_selected.kind != RecoveryKind::A1_LATE_OWNER ||
        late_selected.phase != RecoveryPhase::RETREAT ||
        late_selected.retreat_vehicle_id != intruder.id ||
        late_selected.pass_vehicle_id != late_owner.id ||
        late_selected.retreat_target_s >= late.waiter_stop_s ||
        late_selected.motionFor(intruder.id) != RecoveryMotion::RETREAT ||
        late_selected.motionFor(late_owner.id) != RecoveryMotion::NORMAL) {
        return fail("late-owner recovery did not preserve fixed roles");
    }
    late_vehicles[0].path_s = late_selected.retreat_target_s;
    late_manager.update(late_vehicles, {}, 0.1, false);
    if (late_manager.directive().phase != RecoveryPhase::NONE ||
        late_manager.directive().cooldownActive()) {
        return fail("late-owner recovery did not clear without cooldown");
    }
    std::cout << "deadlock_manager_test: PASS\n";
    return 0;
}
