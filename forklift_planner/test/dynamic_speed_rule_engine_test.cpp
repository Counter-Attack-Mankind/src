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

VehicleAgent diagonalVehicle(int id, double approach, double speed = 0.0) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = 1;
    result.current_speed = speed;
    constexpr double kQuarterPi = 0.7853981633974483;
    result.track.set(RoughPath{
        wp(-approach, -approach, kQuarterPi),
        wp(2.0, 2.0, kQuarterPi)});
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

    // Braking safety remains a direct STOP motion action, but no longer
    // creates ordinary-road holder/waiter ownership.
    RuleEngine emergency_engine(map_param, config);
    std::vector<VehicleAgent> emergency{
        crossingVehicle(0, 0.30, false, config.nominal_speed),
        crossingVehicle(1, 0.30, true, config.nominal_speed)};
    emergency_engine.decide(emergency, 0.1, 15.0);
    const auto emergency_state = emergency_engine.snapshot();
    if (emergency_engine.dynamicSpeedMetrics().emergency_stop_decisions == 0 ||
        !hasDynamicReason(emergency, VehicleAction::STOP) ||
        !emergency_state.reservations.empty()) {
        return fail("braking emergency did not remain reservation-free STOP");
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
    if (!reserved_engine.snapshot().reservations.empty() ||
        reserved_engine.dynamicSpeedMetrics().reservation_deletes == 0 ||
        reserved_engine.dynamicSpeedMetrics().existing_reservation_skips != 0 ||
        reserved_engine.dynamicSpeedMetrics().baseline_conflicts == 0) {
        return fail("ordinary already-inside reservation was not retired");
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

    // Three mutually crossing vehicles exercise all three pairwise dynamic
    // calls. Their requests must aggregate without a legacy reservation.
    RuleEngine multi_engine(map_param, config);
    std::vector<VehicleAgent> multi{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.70, true),
        diagonalVehicle(2, 1.10)};
    multi_engine.decide(multi, 0.1, 15.0);
    const auto& multi_metrics = multi_engine.dynamicSpeedMetrics();
    if (!multi_engine.snapshot().reservations.empty() ||
        multi_metrics.baseline_conflicts < 3 ||
        multi_metrics.reservation_create_multi_vehicle != 0 ||
        multi_engine.lastRollingDynamicDecision().targets.empty()) {
        return fail("three-vehicle pairs did not use dynamic aggregation");
    }
    std::vector<int> target_ids;
    for (const auto& target :
         multi_engine.lastRollingDynamicDecision().targets) {
        for (int id : target_ids) {
            if (id == target.vehicle_id) {
                return fail("multi-pair aggregate stored duplicate target");
            }
        }
        target_ids.push_back(target.vehicle_id);
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
