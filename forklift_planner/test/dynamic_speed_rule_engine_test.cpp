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
    result.track.set(vertical
        ? RoughPath{wp(0.0, -approach, 1.5707963267948966),
                    wp(0.0, 2.0, 1.5707963267948966)}
        : RoughPath{wp(-approach, 0.0, 0.0),
                    wp(2.0, 0.0, 0.0)});
    return result;
}

bool hasDynamicReason(const std::vector<VehicleAgent>& vehicles,
                      VehicleAction action) {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.requested_action == action &&
            vehicle.reason.rfind("dynamic_speed_", 0) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_horizon = 15.0;
    config.prediction_step = 0.05;

    RuleEngine far_engine(map_param, config);
    std::vector<VehicleAgent> far{
        crossingVehicle(0, 2.50, false),
        crossingVehicle(1, 2.90, true)};
    far_engine.decide(far, 0.1, 15.0);
    if (far_engine.dynamicSpeedMetrics().far_decisions == 0 ||
        far[0].requested_action != VehicleAction::NOMINAL ||
        far[1].requested_action != VehicleAction::NOMINAL ||
        !far_engine.snapshot().reservations.empty()) {
        return fail("FAR did not remain reservation-free NOMINAL");
    }

    RuleEngine mid_engine(map_param, config);
    std::vector<VehicleAgent> mid{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    mid_engine.decide(mid, 0.1, 15.0);
    if (mid_engine.dynamicSpeedMetrics().mid_decisions == 0 ||
        !hasDynamicReason(mid, VehicleAction::YIELD) ||
        !mid_engine.snapshot().reservations.empty()) {
        return fail("MID did not accept one reservation-free YIELD");
    }

    RuleEngine near_engine(map_param, config);
    std::vector<VehicleAgent> near{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.79, true)};
    near_engine.decide(near, 0.1, 15.0);
    if (near_engine.dynamicSpeedMetrics().near_decisions == 0 ||
        !hasDynamicReason(near, VehicleAction::CREEP) ||
        !near_engine.snapshot().reservations.empty()) {
        return fail("NEAR did not jump directly to reservation-free CREEP");
    }

    // A close, already-fast loser may select STOP and enter the retained
    // braking-safety reservation path. This is an explicit safety reason,
    // not failure of the selected action to clear 15 s.
    RuleEngine emergency_engine(map_param, config);
    std::vector<VehicleAgent> emergency{
        crossingVehicle(0, 0.30, false, config.nominal_speed),
        crossingVehicle(1, 0.30, true, config.nominal_speed)};
    emergency_engine.decide(emergency, 0.1, 15.0);
    const auto emergency_state = emergency_engine.snapshot();
    if (emergency_engine.dynamicSpeedMetrics().emergency_stop_decisions == 0 ||
        !hasDynamicReason(emergency, VehicleAction::STOP) ||
        emergency_state.reservations.empty() ||
        emergency_state.reservations.begin()->second.create_reason !=
            "braking_safety") {
        return fail("braking emergency did not select STOP/safety reservation");
    }

    RuleEngine reserved_engine(map_param, config);
    std::vector<VehicleAgent> reserved{
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
    reservation.create_reason = "already_inside";
    reservation_state.reservations[{0, 1}] = reservation;
    reserved_engine.restore(reservation_state);
    reserved_engine.decide(reserved, 0.1, 15.0);
    if (reserved_engine.snapshot().reservations.count({0, 1}) == 0 ||
        reserved_engine.dynamicSpeedMetrics().existing_reservation_skips == 0 ||
        hasDynamicReason(reserved, VehicleAction::YIELD) ||
        hasDynamicReason(reserved, VehicleAction::CREEP)) {
        return fail("existing reservation did not skip dynamic selection");
    }

    RuleEngine a1_engine(map_param, config);
    std::vector<VehicleAgent> a1{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    a1[0].a1_departure_committed = true;
    a1[0].a1_departure_priority_until_s = 1.0;
    a1_engine.decide(a1, 0.1, 15.0);
    if (a1_engine.snapshot().reservations.empty() ||
        a1_engine.snapshot().reservations.begin()->second.create_reason !=
            "a1_related" ||
        a1_engine.dynamicSpeedMetrics().a1_fallbacks == 0) {
        return fail("A1 special pair did not retain legacy reservation");
    }

    // Dynamic speed is deliberately two-vehicle-only. With three active
    // vehicles, ordinary pairwise conflicts remain on the legacy path.
    RuleEngine multi_engine(map_param, config);
    std::vector<VehicleAgent> multi{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true),
        crossingVehicle(2, 4.00, false)};
    multi[2].track.set(RoughPath{wp(10.0, 10.0, 0.0),
                                 wp(12.0, 10.0, 0.0)});
    multi_engine.decide(multi, 0.1, 15.0);
    if (multi_engine.snapshot().reservations.empty() ||
        multi_engine.dynamicSpeedMetrics().
                reservation_create_multi_vehicle == 0) {
        return fail("multi-vehicle conflict left legacy reservation path");
    }

    // A clear next rolling period returns to NOMINAL and reports recovery.
    RuleEngine recovery_engine(map_param, config);
    std::vector<VehicleAgent> recovery{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    recovery_engine.decide(recovery, 0.1, 15.0);
    const auto prefix_a = predictTrajectory(
        recovery[0], map_param, config, VehicleAction::NOMINAL, 15.0);
    recovery[0].path_s = prefix_a.back().s;
    recovery[1].track.set(RoughPath{wp(10.0, 10.0, 0.0),
                                    wp(12.0, 10.0, 0.0)});
    recovery[1].path_gen += 1;
    recovery_engine.decide(recovery, 0.1, 15.0);
    if (hasDynamicReason(recovery, VehicleAction::YIELD) ||
        hasDynamicReason(recovery, VehicleAction::CREEP) ||
        !recovery_engine.snapshot().reservations.empty()) {
        return fail("next real rolling decision did not return to NOMINAL");
    }

    std::cout << "dynamic_speed_rule_engine_test: PASS\n";
    return 0;
}
