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

    // A MID action is a rolling 2 s response, not a full-horizon reservation.
    // Even if YIELD does not clear the whole rollout, do not escalate it or
    // reverse the supplied priority inside the current decision.
    std::optional<PairSpeedCoordinationResult> direct_mid;
    std::optional<double> raw_yield_delay;
    for (double speed_a : {0.0, 0.2, 0.4, 0.6}) {
      for (double speed_b : {0.0, 0.2, 0.4, 0.6}) {
       for (double approach_a = 0.5;
             approach_a <= 4.0 && !direct_mid; approach_a += 0.1) {
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
            const auto raw_yield = evaluateSelectedAction(
                a, b, std::vector<PotentialConflictZone>{broadZone(a, b)},
                map_param, config, 15.0, VehicleAction::NOMINAL,
                VehicleAction::YIELD, nominal.event.first_t);
            if (!raw_yield.conflict_free && raw_yield.conflict_delay &&
                *raw_yield.conflict_delay > config.prediction_step) {
                const auto result = evaluate(a, b, map_param, config);
                if (result.action_selected &&
                    result.selected_winner_id == a.id &&
                    result.selected_action_a == VehicleAction::NOMINAL &&
                    result.selected_action_b == VehicleAction::YIELD) {
                    direct_mid = result;
                    raw_yield_delay = raw_yield.conflict_delay;
                }
                break;
            }
        }
       }
      }
    }
    if (!direct_mid || !raw_yield_delay) {
        return fail("remaining MID/YIELD conflict changed direct mapping");
    }

    // Classification metadata must not change generic rolling control.
    PairInteractionResult crossing_label = mid_baseline;
    crossing_label.type = PairInteractionType::CROSSING;
    PairInteractionResult opposing_label = mid_baseline;
    opposing_label.type = PairInteractionType::OPPOSING;
    opposing_label.shared_segment.valid = true;
    opposing_label.shared_segment.s_a_enter = 0.0;
    opposing_label.shared_segment.s_a_exit = mid_a.track.length();
    opposing_label.shared_segment.s_b_enter = 0.0;
    opposing_label.shared_segment.s_b_exit = mid_b.track.length();
    PairInteractionResult following_label = mid_baseline;
    following_label.type = PairInteractionType::SAME_DIRECTION;
    const auto crossing_result = evaluatePairSpeedCoordination(
        mid_a, mid_b, {}, crossing_label, map_param, config, 15.0, mid_a.id);
    const auto opposing_result = evaluatePairSpeedCoordination(
        mid_a, mid_b, {}, opposing_label, map_param, config, 15.0, mid_a.id);
    const auto following_result = evaluatePairSpeedCoordination(
        mid_a, mid_b, {}, following_label, map_param, config, 15.0, mid_a.id);
    const auto sameControl = [&](const PairSpeedCoordinationResult& result) {
        return result.selected_winner_id == crossing_result.selected_winner_id &&
               result.selected_action_a == crossing_result.selected_action_a &&
               result.selected_action_b == crossing_result.selected_action_b &&
               result.reason == crossing_result.reason;
    };
    if (!sameControl(opposing_result) || !sameControl(following_result)) {
        return fail("interaction label still changes generic pair control");
    }

    const auto fixed_order = evaluatePairSpeedCoordination(
        mid_a, mid_b, {}, mid_baseline, map_param, config, 15.0, mid_b.id);
    if (fixed_order.selected_winner_id != mid_b.id ||
        fixed_order.selected_action_a != VehicleAction::YIELD ||
        fixed_order.selected_action_b != VehicleAction::NOMINAL) {
        return fail("generic coordinator changed supplied priority order");
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

    const auto base_stop = evaluateTtcStopBoundary(
        1.3, VehicleAction::CREEP, config);
    const double expected_threshold =
        config.rolling_refresh_period +
        config.nominal_speed * config.creep_ratio / config.max_decel +
        config.dynamic_stop_time_margin;
    if (!near(base_stop.stop_threshold, expected_threshold) ||
        !base_stop.stop_required ||
        evaluateTtcStopBoundary(3.8, VehicleAction::CREEP, config).
            stop_required ||
        !evaluateTtcStopBoundary(expected_threshold, VehicleAction::CREEP,
                                 config).stop_required) {
        return fail("TTC STOP boundary value or inclusive edge changed");
    }

    MultiVehicleConfig shorter_refresh = config;
    shorter_refresh.rolling_refresh_period = 1.0;
    if (!evaluateTtcStopBoundary(1.5, VehicleAction::CREEP, config).
            stop_required ||
        evaluateTtcStopBoundary(1.5, VehicleAction::CREEP,
                                shorter_refresh).stop_required) {
        return fail("rolling refresh period did not move TTC STOP boundary");
    }

    MultiVehicleConfig faster_creep = config;
    faster_creep.creep_ratio = 0.50;
    if (evaluateTtcStopBoundary(2.35, VehicleAction::CREEP, config).
            stop_required ||
        !evaluateTtcStopBoundary(2.35, VehicleAction::CREEP,
                                 faster_creep).stop_required) {
        return fail("creep ratio did not move TTC STOP boundary");
    }

    MultiVehicleConfig weaker_brake = config;
    weaker_brake.max_decel = 0.10;
    if (evaluateTtcStopBoundary(2.40, VehicleAction::CREEP, config).
            stop_required ||
        !evaluateTtcStopBoundary(2.40, VehicleAction::CREEP,
                                 weaker_brake).stop_required) {
        return fail("max deceleration did not move TTC STOP boundary");
    }

    // Braking infeasibility stops only the yielding side; the supplied stable
    // priority remains nominal and no alternate order is searched.
    VehicleAgent immediate_a = crossingVehicle(10, 0.0, false, 0.0);
    VehicleAgent immediate_b = crossingVehicle(11, 0.0, false, 0.0);
    immediate_b.track.set(RoughPath{
        wp(0.0, 0.0, 3.14159265358979323846),
        wp(-2.0, 0.0, 3.14159265358979323846)});
    PairInteractionResult immediate_baseline;
    immediate_baseline.type = PairInteractionType::OPPOSING;
    immediate_baseline.event.valid = true;
    immediate_baseline.event.first_t = 0.0;
    immediate_baseline.shared_segment.valid = true;
    immediate_baseline.shared_segment.s_a_enter = 0.0;
    immediate_baseline.shared_segment.s_a_exit = 1.0;
    immediate_baseline.shared_segment.s_b_enter = 0.0;
    immediate_baseline.shared_segment.s_b_exit = 1.0;
    const auto unresolved = evaluatePairSpeedCoordination(
        immediate_a, immediate_b, {}, immediate_baseline, map_param,
        config, 15.0, immediate_b.id, true);
    if (unresolved.selected_action_a != VehicleAction::STOP ||
        unresolved.selected_action_b != VehicleAction::NOMINAL ||
        unresolved.selected_winner_id != immediate_b.id ||
        unresolved.reason != "rolling_emergency_stop") {
        return fail("braking emergency changed stable-priority response");
    }

    std::cout << "[MID-DIRECT] baseline_first_t="
              << *direct_mid->original_first_conflict_t
              << " raw_yield_delay=" << *raw_yield_delay
              << " selected_action="
              << static_cast<int>(direct_mid->selected_action_b) << '\n';
    std::cout << "dynamic_speed_coordination_test: PASS\n";
    return 0;
}
