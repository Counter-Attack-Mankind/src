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
    PotentialConflictZone zone;
    zone.s_self_enter = 0.85; zone.s_self_exit = 1.15;
    zone.s_other_enter = 0.85; zone.s_other_exit = 1.15;
    geometry.zones.push_back(zone);

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
    vehicles[0].path_s = selected.retreat_target_s;
    manager.update(vehicles, {geometry}, 0.1, false);
    if (manager.directive().phase != RecoveryPhase::PASS ||
        !manager.passOverride(0, 1)) {
        return fail("RETREAT did not transition to pair-local PASS");
    }
    vehicles[1].path_s = manager.directive().pass_clear_s;
    manager.update(vehicles, {geometry}, 0.1, false);
    if (manager.directive().phase != RecoveryPhase::CLEAR) {
        return fail("PASS did not transition to CLEAR");
    }
    std::cout << "deadlock_manager_test: PASS\n";
    return 0;
}
