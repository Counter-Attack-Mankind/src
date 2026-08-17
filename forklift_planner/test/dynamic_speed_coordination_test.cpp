#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

int fail(const std::string& message) {
    std::cerr << "dynamic_speed_coordination_test: " << message << '\n';
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

PotentialConflictZone broadZone(const VehicleAgent& a,
                                const VehicleAgent& b) {
    PotentialConflictZone zone;
    zone.s_self_enter = 0.0;
    zone.s_self_exit = a.track.length();
    zone.s_other_enter = 0.0;
    zone.s_other_exit = b.track.length();
    return zone;
}

PairInteractionResult baseline(const VehicleAgent& a, const VehicleAgent& b,
                               const MapParam& map_param,
                               const MultiVehicleConfig& config,
                               double horizon) {
    const std::vector<PotentialConflictZone> zones{broadZone(a, b)};
    return detectPairInteractionFromPredictions(
        a, b, zones,
        predictTrajectory(a, map_param, config, VehicleAction::NOMINAL,
                          horizon),
        predictTrajectory(b, map_param, config, VehicleAction::NOMINAL,
                          horizon));
}

PairSpeedCoordinationResult evaluate(
    const VehicleAgent& a, const VehicleAgent& b,
    const MapParam& map_param, const MultiVehicleConfig& config,
    bool emergency_stop = false) {
    const auto nominal = baseline(a, b, map_param, config, 15.0);
    return evaluatePairSpeedCoordination(
        a, b, std::vector<PotentialConflictZone>{broadZone(a, b)},
        nominal, map_param, config, 15.0, 0, emergency_stop);
}

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_horizon = 15.0;
    config.prediction_step = 0.05;

    constexpr double eps = 1e-6;
    if (classifyDynamicInterventionBand(10.0, config) !=
            DynamicInterventionBand::FAR ||
        classifyDynamicInterventionBand(10.0 - eps, config) !=
            DynamicInterventionBand::MID ||
        classifyDynamicInterventionBand(5.0, config) !=
            DynamicInterventionBand::MID ||
        classifyDynamicInterventionBand(5.0 - eps, config) !=
            DynamicInterventionBand::NEAR) {
        return fail("configured FAR/MID/NEAR boundaries changed");
    }
    if (selectRollingSpeedAction(DynamicInterventionBand::FAR, false) !=
            VehicleAction::NOMINAL ||
        selectRollingSpeedAction(DynamicInterventionBand::MID, false) !=
            VehicleAction::YIELD ||
        selectRollingSpeedAction(DynamicInterventionBand::NEAR, false) !=
            VehicleAction::CREEP ||
        selectRollingSpeedAction(DynamicInterventionBand::FAR, true) !=
            VehicleAction::STOP) {
        return fail("rolling band-to-action mapping changed");
    }

    const VehicleAgent far_a = crossingVehicle(0, 2.50, false);
    const VehicleAgent far_b = crossingVehicle(1, 2.90, true);
    const auto far_result = evaluate(far_a, far_b, map_param, config);
    if (!far_result.action_selected ||
        far_result.selected_action_a != VehicleAction::NOMINAL ||
        far_result.selected_action_b != VehicleAction::NOMINAL) {
        return fail("FAR did not select NOMINAL/NOMINAL");
    }

    const VehicleAgent mid_a = crossingVehicle(0, 1.50, false);
    const VehicleAgent mid_b = crossingVehicle(1, 1.90, true);
    const auto mid_baseline = baseline(mid_a, mid_b, map_param, config, 15.0);
    const auto mid_result = evaluate(mid_a, mid_b, map_param, config);
    if (!mid_baseline.event.valid ||
        classifyDynamicInterventionBand(mid_baseline.event.first_t, config) !=
            DynamicInterventionBand::MID ||
        !mid_result.action_selected ||
        mid_result.selected_action_a != VehicleAction::NOMINAL ||
        mid_result.selected_action_b != VehicleAction::YIELD) {
        return fail("MID did not select exactly one YIELD target");
    }

    const VehicleAgent near_a = crossingVehicle(0, 0.30, false);
    const VehicleAgent near_b = crossingVehicle(1, 0.55, true);
    const auto near_baseline = baseline(
        near_a, near_b, map_param, config, 15.0);
    const auto near_result = evaluate(near_a, near_b, map_param, config);
    if (!near_baseline.event.valid ||
        classifyDynamicInterventionBand(near_baseline.event.first_t, config) !=
            DynamicInterventionBand::NEAR ||
        near_result.selected_action_b != VehicleAction::CREEP) {
        return fail("NEAR did not jump directly to CREEP");
    }

    const auto emergency = evaluate(
        mid_a, mid_b, map_param, config, true);
    if (!emergency.emergency_stop ||
        emergency.selected_action_b != VehicleAction::STOP) {
        return fail("emergency did not directly select STOP");
    }

    // Find a deterministic MID case where the one selected YIELD action
    // delays, but does not clear, the full 15 s conflict. It must still be
    // accepted as this rolling period's action.
    std::optional<PairSpeedCoordinationResult> delayed_mid;
    for (double speed_a : {0.0, 0.2, 0.4, 0.6}) {
      for (double speed_b : {0.0, 0.2, 0.4, 0.6}) {
       for (double approach_a = 0.5;
            approach_a <= 4.0 && !delayed_mid; approach_a += 0.1) {
        for (double approach_b = 0.5;
             approach_b <= 4.0; approach_b += 0.1) {
            const VehicleAgent a = crossingVehicle(
                0, approach_a, false, speed_a);
            const VehicleAgent b = crossingVehicle(
                1, approach_b, true, speed_b);
            const auto nominal = baseline(a, b, map_param, config, 15.0);
            if (!nominal.event.valid ||
                classifyDynamicInterventionBand(nominal.event.first_t,
                                                config) !=
                    DynamicInterventionBand::MID) {
                continue;
            }
            const auto result = evaluate(a, b, map_param, config);
            if (result.action_selected &&
                result.selected_action_b == VehicleAction::YIELD &&
                !result.evaluation.conflict_free &&
                result.evaluation.first_conflict_t &&
                result.evaluation.conflict_delay &&
                *result.evaluation.conflict_delay > config.prediction_step) {
                delayed_mid = result;
                break;
            }
        }
       }
      }
    }
    if (!delayed_mid) {
        return fail("no delayed-but-remaining MID/YIELD fixture found");
    }

    VehicleAgent low = crossingVehicle(2, 1.0, false, 0.0);
    const auto nominal_prediction = predictTrajectory(
        low, map_param, config, VehicleAction::NOMINAL, 0.1);
    if (nominal_prediction.size() < 2 ||
        !near(nominal_prediction[1].speed,
              config.max_accel * config.prediction_step)) {
        return fail("NOMINAL acceleration constraint changed");
    }
    VehicleAgent fast = crossingVehicle(
        3, 1.0, false, config.nominal_speed);
    const auto yielding = predictTrajectory(
        fast, map_param, config, VehicleAction::YIELD, 0.1);
    if (yielding.size() < 2 ||
        !near(yielding[1].speed,
              config.nominal_speed -
                  config.max_decel * config.prediction_step)) {
        return fail("YIELD deceleration constraint changed");
    }

    const VehicleAgent before_a = mid_a;
    const VehicleAgent before_b = mid_b;
    for (int repeat = 0; repeat < 3; ++repeat) {
        (void)evaluate(mid_a, mid_b, map_param, config);
    }
    if (!near(mid_a.path_s, before_a.path_s) ||
        !near(mid_a.current_speed, before_a.current_speed) ||
        mid_a.action != before_a.action ||
        !near(mid_b.path_s, before_b.path_s) ||
        !near(mid_b.current_speed, before_b.current_speed) ||
        mid_b.action != before_b.action) {
        return fail("selected-action evaluation changed live input");
    }

    VehicleAgent near_entry = crossingVehicle(6, 0.2, false, 0.2);
    if (!hasInsufficientBrakingMargin(
            near_entry, 0.25, config, 0.1)) {
        return fail("insufficient braking margin was not detected");
    }

    std::cout << "[MID-DELAY] baseline_first_t="
              << *delayed_mid->original_first_conflict_t
              << " selected=YIELD after_first_t="
              << *delayed_mid->evaluation.first_conflict_t
              << " delay=" << *delayed_mid->evaluation.conflict_delay
              << " accepted=true\n";
    std::cout << "dynamic_speed_coordination_test: PASS\n";
    return 0;
}
