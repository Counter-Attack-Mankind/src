#include "forklift_planner/multi_vehicle/rule_engine.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

int fail(const std::string& message) {
    std::cerr << "dynamic_speed_rule_engine_test: " << message << '\n';
    return 1;
}

RoughWp wp(double x, double y, double theta) {
    return RoughWp{x, y, theta, WpType::FORWARD};
}

VehicleAgent crossingVehicle(int id, double approach, bool vertical,
                             double speed = 0.0) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = 1;
    result.current_speed = speed;
    if (vertical) {
        result.track.set(RoughPath{
            wp(0.0, -approach, 1.5707963267948966),
            wp(0.0, 2.0, 1.5707963267948966)});
    } else {
        result.track.set(RoughPath{
            wp(-approach, 0.0, 0.0), wp(2.0, 0.0, 0.0)});
    }
    return result;
}

bool hasDynamicReason(const std::vector<VehicleAgent>& vehicles) {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.reason.rfind("dynamic_speed_", 0) == 0) return true;
    }
    return false;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_horizon = 15.0;
    config.prediction_step = 0.05;

    // 9. Pure counterfactual calls cannot mutate a seeded RuleEngine state.
    RuleEngine side_effect_engine(map_param, config);
    RuleEngine::SimSnapshot seeded;
    RuleEngine::ConflictReservation sentinel;
    sentinel.owner_id = 9;
    seeded.reservations[{8, 9}] = sentinel;
    seeded.tokens.grant(77, 8, 1.0);
    seeded.now = 4.0;
    side_effect_engine.restore(seeded);
    VehicleAgent pure_a = crossingVehicle(0, 0.30, false);
    VehicleAgent pure_b = crossingVehicle(1, 0.70, true);
    const PairInteractionResult pure_baseline =
        side_effect_engine.detectPairInteraction(pure_a, pure_b, 15.0);
    (void)evaluatePairSpeedCoordination(
        pure_a, pure_b, pure_baseline.potential_zones, pure_baseline,
        map_param, config, 15.0, 0);
    const RuleEngine::SimSnapshot after_pure = side_effect_engine.snapshot();
    if (after_pure.reservations.size() != seeded.reservations.size() ||
        after_pure.tokens.holder(77) != 8 ||
        std::abs(after_pure.now - seeded.now) > 1e-12) {
        return fail("counterfactual evaluation changed RuleEngine state");
    }

    // 10. A valid existing reservation remains on the legacy path.
    RuleEngine reserved_engine(map_param, config);
    std::vector<VehicleAgent> reserved_vehicles{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    RuleEngine::SimSnapshot reservation_state;
    RuleEngine::ConflictReservation reservation;
    reservation.owner_id = 0;
    reservation.gen_lo = 1;
    reservation.gen_hi = 1;
    reservation.enter_lo = 0.10;
    reservation.exit_lo = 0.70;
    reservation.enter_hi = 0.40;
    reservation.exit_hi = 1.00;
    reservation_state.reservations[{0, 1}] = reservation;
    reserved_engine.restore(reservation_state);
    reserved_engine.decide(reserved_vehicles, 0.1, 15.0);
    const auto reserved_after = reserved_engine.snapshot();
    if (reserved_after.reservations.count({0, 1}) == 0 ||
        reserved_after.reservations.at({0, 1}).owner_id != 0 ||
        reserved_engine.dynamicSpeedMetrics().existing_reservation_skips == 0 ||
        hasDynamicReason(reserved_vehicles)) {
        return fail("existing reservation left the legacy chain");
    }

    // 11. An active A1 departure-protected pair bypasses ordinary dynamic
    // speed search and retains legacy reservation arbitration.
    RuleEngine a1_engine(map_param, config);
    std::vector<VehicleAgent> a1_vehicles{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    a1_vehicles[0].a1_departure_committed = true;
    a1_vehicles[0].a1_departure_priority_until_s = 1.0;
    a1_engine.decide(a1_vehicles, 0.1, 15.0);
    if (a1_engine.dynamicSpeedMetrics().a1_fallbacks == 0 ||
        a1_engine.snapshot().reservations.empty() ||
        hasDynamicReason(a1_vehicles)) {
        return fail("A1 protected pair entered ordinary dynamic search");
    }

    // 12. A near conflict cannot substitute speed shaping for the legacy
    // reservation/STOP safety fallback.
    RuleEngine near_engine(map_param, config);
    std::vector<VehicleAgent> near_vehicles{
        crossingVehicle(0, 0.30, false, config.nominal_speed),
        crossingVehicle(1, 0.30, true, config.nominal_speed)};
    near_engine.decide(near_vehicles, 0.1, 15.0);
    if (near_engine.dynamicSpeedMetrics().near_fallbacks == 0 ||
        near_engine.snapshot().reservations.empty() ||
        hasDynamicReason(near_vehicles)) {
        return fail("near conflict did not use legacy fallback");
    }

    // 13. Cycle 1 selects a temporary YIELD. After executing only a 2 s
    // prefix, cycle 2 starts from the new true speed with a NOMINAL baseline;
    // when that baseline is clear, the dynamic request returns to NOMINAL.
    // A later hard rule may still request a stronger action.
    RuleEngine recovery_engine(map_param, config);
    std::vector<VehicleAgent> recovery{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.79, true)};
    recovery_engine.decide(recovery, 0.1, 15.0);
    if (recovery[1].requested_action != VehicleAction::YIELD ||
        recovery[1].reason.rfind("dynamic_speed_", 0) != 0 ||
        !recovery_engine.snapshot().reservations.empty()) {
        return fail("cycle 1 did not select reservation-free YIELD");
    }
    const auto prefix_a = predictTrajectory(
        recovery[0], map_param, config, VehicleAction::NOMINAL, 2.0);
    const auto prefix_b = predictTrajectory(
        recovery[1], map_param, config, VehicleAction::YIELD, 2.0);
    recovery[0].path_s = prefix_a.back().s;
    recovery[0].current_speed = prefix_a.back().speed;
    recovery[1].path_s = prefix_b.back().s;
    recovery[1].current_speed = prefix_b.back().speed;
    recovery[1].action = VehicleAction::YIELD;
    const PairInteractionResult next_baseline =
        recovery_engine.detectPairInteraction(recovery[0], recovery[1], 15.0);
    if (next_baseline.event.valid) {
        return fail("chosen recovery fixture still conflicts after 2 s prefix");
    }
    recovery_engine.decide(recovery, 0.1, 15.0);
    if (recovery_engine.dynamicSpeedMetrics().nominal_recoveries == 0 ||
        hasDynamicReason(recovery) ||
        !recovery_engine.snapshot().reservations.empty()) {
        return fail("next rolling cycle did not restore NOMINAL target");
    }

    std::cout << "dynamic_speed_rule_engine_test: PASS\n";
    return 0;
}
