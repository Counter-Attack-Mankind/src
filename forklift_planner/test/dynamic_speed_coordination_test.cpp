#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <cmath>
#include <iostream>
#include <string>

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
    RoughPath path;
    if (vertical) {
        path = {wp(0.0, -approach, 1.5707963267948966),
                wp(0.0, 2.0, 1.5707963267948966)};
    } else {
        path = {wp(-approach, 0.0, 0.0), wp(2.0, 0.0, 0.0)};
    }
    result.track.set(path);
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

struct Scenario {
    VehicleAgent a;
    VehicleAgent b;
    PairInteractionResult baseline;
    PairSpeedCoordinationResult result;
};

Scenario makeScenario(
    const MapParam& map_param, const MultiVehicleConfig& config,
    double approach_a, double approach_b) {
    constexpr double horizon = 15.0;
    VehicleAgent a = crossingVehicle(0, approach_a, false);
    VehicleAgent b = crossingVehicle(1, approach_b, true);
    PairInteractionResult nominal =
        baseline(a, b, map_param, config, horizon);
    const std::vector<PotentialConflictZone> zones{broadZone(a, b)};
    PairSpeedCoordinationResult result = evaluatePairSpeedCoordination(
        a, b, zones, nominal, map_param, config, horizon, 0);
    return Scenario{a, b, nominal, result};
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

    // Phase 2.2 intervention bands. Boundaries are intentionally inclusive
    // on the less urgent side: exactly 10 s is FAR and exactly 5 s is MID.
    constexpr double eps = 1e-6;
    if (classifyDynamicInterventionBand(
            10.0, config) != DynamicInterventionBand::FAR ||
        classifyDynamicInterventionBand(
            10.0 - eps, config) != DynamicInterventionBand::MID ||
        classifyDynamicInterventionBand(
            10.0 + eps, config) != DynamicInterventionBand::FAR ||
        classifyDynamicInterventionBand(
            5.0, config) != DynamicInterventionBand::MID ||
        classifyDynamicInterventionBand(
            5.0 - eps, config) != DynamicInterventionBand::NEAR ||
        classifyDynamicInterventionBand(
            5.0 + eps, config) != DynamicInterventionBand::MID) {
        return fail("FAR/MID/NEAR boundary classification changed");
    }

    // 1. NOMINAL/NOMINAL conflicts while NOMINAL/YIELD clears the horizon.
    const Scenario yield_clear = makeScenario(
        map_param, config, 0.30, 0.70);
    if (!yield_clear.baseline.event.valid ||
        !yield_clear.result.solved_by_speed_adjustment ||
        yield_clear.result.selected_action_a != VehicleAction::NOMINAL ||
        yield_clear.result.selected_action_b != VehicleAction::YIELD) {
        return fail("no deterministic YIELD-clear crossing was found");
    }

    // 2. YIELD still conflicts, then CREEP is the first clear candidate.
    const Scenario creep_clear = makeScenario(
        map_param, config, 0.30, 0.55);
    if (creep_clear.result.candidates.size() != 2 ||
        creep_clear.result.candidates[0].conflict_free ||
        !creep_clear.result.candidates[1].conflict_free ||
        creep_clear.result.selected_action_b != VehicleAction::CREEP) {
        return fail("progressive YIELD-to-CREEP search failed");
    }

    // 3. An overlap at t=0 cannot be repaired by either speed target.
    VehicleAgent overlap_a = crossingVehicle(0, 0.0, false);
    VehicleAgent overlap_b = crossingVehicle(1, 0.0, true);
    const PairInteractionResult overlap_baseline =
        baseline(overlap_a, overlap_b, map_param, config, 15.0);
    const std::vector<PotentialConflictZone> overlap_zones{
        broadZone(overlap_a, overlap_b)};
    const PairSpeedCoordinationResult all_failed =
        evaluatePairSpeedCoordination(
            overlap_a, overlap_b, overlap_zones, overlap_baseline,
            map_param, config, 15.0, 0);
    if (!all_failed.fallback_required ||
        all_failed.solved_by_speed_adjustment ||
        all_failed.candidates.size() != 2) {
        return fail("unavoidable overlap did not request legacy fallback");
    }

    // 4. A candidate that merely delays first_t is still a failure.
    if (!creep_clear.result.candidates[0].first_conflict_t ||
        !creep_clear.result.original_first_conflict_t ||
        *creep_clear.result.candidates[0].first_conflict_t <=
            *creep_clear.result.original_first_conflict_t ||
        creep_clear.result.candidates[0].conflict_free) {
        return fail("delayed but remaining conflict was accepted");
    }

    // 5. Low speed with NOMINAL accelerates under max_accel.
    VehicleAgent low = crossingVehicle(2, 1.0, false, 0.0);
    const auto nominal = predictTrajectory(
        low, map_param, config, VehicleAction::NOMINAL, 0.1);
    if (nominal.size() < 2 ||
        !near(nominal[1].speed,
              config.max_accel * config.prediction_step)) {
        return fail("NOMINAL acceleration constraint changed");
    }

    // 6. High speed with YIELD decelerates under max_decel, not instantly.
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

    // 7. A tight curve caps even a NOMINAL target.
    RoughPath arc;
    constexpr double radius = 0.5;
    for (int index = 0; index <= 100; ++index) {
        const double angle = 1.5707963267948966 * index / 100.0;
        arc.push_back(wp(radius * std::cos(angle),
                         radius * std::sin(angle),
                         angle + 1.5707963267948966));
    }
    MultiVehicleConfig curve_config = config;
    curve_config.lat_accel_max = 0.001;
    VehicleAgent curved;
    curved.id = 4;
    curved.mode = VehicleMode::ACTIVE;
    curved.track.set(arc);
    curved.path_s = 0.2;
    curved.current_speed = config.nominal_speed;
    const auto curve_prediction = predictTrajectory(
        curved, map_param, curve_config, VehicleAction::NOMINAL, 0.1);
    if (curve_prediction.size() < 2 ||
        !(curve_prediction[1].speed < config.nominal_speed)) {
        return fail("curvature cap was bypassed by action target");
    }

    // 8. STOP prediction clamps at the path endpoint and never overshoots.
    VehicleAgent endpoint = crossingVehicle(5, 0.005, false, 0.2);
    endpoint.track.set(RoughPath{wp(0.0, 0.0, 0.0),
                                 wp(0.005, 0.0, 0.0)});
    const auto stopped = predictTrajectory(
        endpoint, map_param, config, VehicleAction::STOP, 1.0);
    if (stopped.empty() ||
        stopped.back().s > endpoint.track.length() + 1e-12 ||
        !near(stopped.back().speed, 0.0)) {
        return fail("STOP/path-end constraint failed");
    }

    // 9. Repeated candidate evaluation changes neither live vehicle input.
    const VehicleAgent before_a = yield_clear.a;
    const VehicleAgent before_b = yield_clear.b;
    for (int repeat = 0; repeat < 3; ++repeat) {
        (void)evaluatePairSpeedCoordination(
            yield_clear.a, yield_clear.b,
            std::vector<PotentialConflictZone>{
                broadZone(yield_clear.a, yield_clear.b)},
            yield_clear.baseline, map_param, config, 15.0, 0);
    }
    if (!near(yield_clear.a.path_s, before_a.path_s) ||
        !near(yield_clear.a.current_speed, before_a.current_speed) ||
        yield_clear.a.action != before_a.action ||
        yield_clear.a.reason != before_a.reason ||
        !near(yield_clear.b.path_s, before_b.path_s) ||
        !near(yield_clear.b.current_speed, before_b.current_speed) ||
        yield_clear.b.action != before_b.action ||
        yield_clear.b.reason != before_b.reason) {
        return fail("counterfactual evaluation changed live input");
    }

    // 12. Near conflicts conservatively use legacy fallback.
    VehicleAgent near_entry = crossingVehicle(6, 0.2, false, 0.2);
    if (!hasInsufficientBrakingMargin(
            near_entry, 0.25, config, 0.1)) {
        return fail("insufficient braking margin was not detected");
    }
    VehicleAgent far_entry = crossingVehicle(7, 1.0, false, 0.2);
    if (hasInsufficientBrakingMargin(
            far_entry, 1.0, config, 0.1)) {
        return fail("distant conflict was misclassified as near");
    }

    std::cout << "yield_clear baseline_t="
              << *yield_clear.result.original_first_conflict_t
              << " approach_a=" << -yield_clear.a.track.path().front().x
              << " approach_b=" << -yield_clear.b.track.path().front().y
              << '\n';
    std::cout << "creep_clear baseline_t="
              << *creep_clear.result.original_first_conflict_t
              << " yield_t="
              << *creep_clear.result.candidates[0].first_conflict_t
              << " approach_a=" << -creep_clear.a.track.path().front().x
              << " approach_b=" << -creep_clear.b.track.path().front().y
              << '\n';
    std::cout << "dynamic_speed_coordination_test: PASS\n";
    return 0;
}
