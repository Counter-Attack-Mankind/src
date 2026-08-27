#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <algorithm>
#include <cmath>

#include "forklift_planner/multi_vehicle/bridge_ttc_correction.h"

namespace forklift_planner {
namespace multi_vehicle {

DynamicInterventionBand classifyDynamicInterventionBand(
    double vehicle_ttc, const MultiVehicleConfig& config) {
    if (vehicle_ttc >= config.dynamic_speed_far_threshold) {
        return DynamicInterventionBand::FAR;
    }
    if (vehicle_ttc >= config.dynamic_speed_near_threshold) {
        return DynamicInterventionBand::MID;
    }
    return DynamicInterventionBand::NEAR;
}

const char* dynamicInterventionBandName(DynamicInterventionBand band) {
    switch (band) {
        case DynamicInterventionBand::NEAR: return "NEAR";
        case DynamicInterventionBand::MID: return "MID";
        case DynamicInterventionBand::FAR: return "FAR";
    }
    return "UNKNOWN";
}

VehicleAction selectRollingSpeedAction(
    DynamicInterventionBand band, bool emergency_stop) {
    if (emergency_stop) return VehicleAction::STOP;
    switch (band) {
        case DynamicInterventionBand::FAR:
            return VehicleAction::NOMINAL;
        case DynamicInterventionBand::MID:
            return VehicleAction::YIELD;
        case DynamicInterventionBand::NEAR:
            return VehicleAction::CREEP;
    }
    return VehicleAction::STOP;
}

SelectedSpeedActionEvaluation evaluateSelectedAction(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, VehicleAction action_a,
    VehicleAction action_b,
    std::optional<double> original_first_overlap_t) {
    SelectedSpeedActionEvaluation evaluation;
    evaluation.action_a = action_a;
    evaluation.action_b = action_b;
    const auto prediction_a = predictTrajectory(
        vehicle_a, map_param, config, action_a, prediction_horizon);
    const auto prediction_b = predictTrajectory(
        vehicle_b, map_param, config, action_b, prediction_horizon);
    // Candidate validation deliberately uses the same synchronized physical
    // OBB detector as the nominal baseline. Interaction labels are not a
    // separate control authority.
    const PairInteractionResult interaction =
        detectPairInteractionFromPredictions(
            vehicle_a, vehicle_b, potential_zones,
            prediction_a, prediction_b);
    evaluation.conflict_free = !interaction.event.valid;
    if (interaction.event.valid) {
        const PairBridgeTtcCorrection bridge = evaluateBridgeTtcCorrection(
            vehicle_a, vehicle_b, prediction_a, prediction_b, interaction,
            map_param, config);
        evaluation.first_overlap_t = interaction.event.first_overlap_t;
        evaluation.collision_s_a = interaction.event.collision_s_a;
        evaluation.collision_s_b = interaction.event.collision_s_b;
        evaluation.danger_s_a = bridge.a.near_boundary_s;
        evaluation.danger_s_b = bridge.b.near_boundary_s;
        evaluation.ttc_a = bridge.a.corrected_ttc;
        evaluation.ttc_b = bridge.b.corrected_ttc;
        if (original_first_overlap_t) {
            evaluation.conflict_delay =
                interaction.event.first_overlap_t -
                *original_first_overlap_t;
        }
    }
    return evaluation;
}

PairSpeedCoordinationResult evaluatePairSpeedCoordination(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, int preferred_winner_id) {
    PairSpeedCoordinationResult result;
    if (!nominal_baseline.event.valid) {
        result.reason = "baseline_clear";
        return result;
    }
    result.first_overlap_t = nominal_baseline.event.first_overlap_t;
    result.baseline_ttc_a = nominal_baseline.event.ttc_a;
    result.baseline_ttc_b = nominal_baseline.event.ttc_b;
    if (preferred_winner_id != vehicle_a.id &&
        preferred_winner_id != vehicle_b.id) {
        result.reason = "no_priority_winner";
        return result;
    }

    result.attempted = true;
    result.action_selected = true;
    result.selected_winner_id = preferred_winner_id;
    const bool a_is_priority = preferred_winner_id == vehicle_a.id;
    result.yielding_ttc = a_is_priority
        ? nominal_baseline.event.ttc_b : nominal_baseline.event.ttc_a;
    result.yielding_band = classifyDynamicInterventionBand(
        *result.yielding_ttc, config);
    VehicleAction yielding_action = selectRollingSpeedAction(
        result.yielding_band, false);
    const TtcStopBoundary baseline_yielding_boundary =
        evaluateTtcStopBoundary(
            *result.yielding_ttc, yielding_action, config);
    result.yielding_stop_threshold =
        baseline_yielding_boundary.stop_threshold;
    if (baseline_yielding_boundary.stop_required) {
        result.yielding_safety_stop = true;
        yielding_action = VehicleAction::STOP;
    }
    const VehicleAction residual_action_a = a_is_priority
        ? VehicleAction::NOMINAL : yielding_action;
    const VehicleAction residual_action_b = a_is_priority
        ? yielding_action : VehicleAction::NOMINAL;
    const SelectedSpeedActionEvaluation residual = evaluateSelectedAction(
        vehicle_a, vehicle_b, potential_zones, map_param, config,
        prediction_horizon, residual_action_a, residual_action_b);
    result.residual_evaluated = true;
    result.residual_conflict = !residual.conflict_free;
    result.residual_first_overlap_t = residual.first_overlap_t;
    result.residual_ttc_a = residual.ttc_a;
    result.residual_ttc_b = residual.ttc_b;
    if (residual.ttc_a && residual.ttc_b) {
        result.residual_priority_ttc = a_is_priority
            ? residual.ttc_a : residual.ttc_b;
        result.residual_yielding_ttc = a_is_priority
            ? residual.ttc_b : residual.ttc_a;
        const TtcStopBoundary priority_boundary = evaluateTtcStopBoundary(
            *result.residual_priority_ttc,
            VehicleAction::NOMINAL, config);
        result.priority_stop_threshold = priority_boundary.stop_threshold;
        result.priority_safety_stop = priority_boundary.stop_required;
        if (yielding_action != VehicleAction::STOP) {
            const TtcStopBoundary residual_yielding_boundary =
                evaluateTtcStopBoundary(
                    *result.residual_yielding_ttc,
                    yielding_action, config);
            result.yielding_stop_threshold =
                residual_yielding_boundary.stop_threshold;
            if (residual_yielding_boundary.stop_required) {
                result.yielding_safety_stop = true;
                yielding_action = VehicleAction::STOP;
            }
        }
    }
    const VehicleAction priority_action = result.priority_safety_stop
        ? VehicleAction::STOP : VehicleAction::NOMINAL;
    result.selected_action_a = a_is_priority
        ? priority_action : yielding_action;
    result.selected_action_b = a_is_priority
        ? yielding_action : priority_action;

    result.emergency_stop = result.yielding_safety_stop ||
                            result.priority_safety_stop;
    if (result.emergency_stop) {
        result.reason = "rolling_emergency_stop";
    } else if (result.yielding_band == DynamicInterventionBand::FAR) {
        result.reason = "rolling_far_nominal";
    } else if (result.yielding_band == DynamicInterventionBand::MID) {
        result.reason = "rolling_mid_yield";
    } else {
        result.reason = "rolling_near_creep";
    }
    return result;
}

TtcStopBoundary evaluateTtcStopBoundary(
    double vehicle_ttc, VehicleAction planned_action,
    const MultiVehicleConfig& config) {
    TtcStopBoundary result;
    result.planned_action = planned_action;
    switch (planned_action) {
        case VehicleAction::STOP:
            result.action_speed = 0.0;
            break;
        case VehicleAction::CREEP:
            result.action_speed = config.nominal_speed * config.creep_ratio;
            break;
        case VehicleAction::YIELD:
            result.action_speed = config.nominal_speed * config.yield_ratio;
            break;
        case VehicleAction::NOMINAL:
            result.action_speed = config.nominal_speed;
            break;
        case VehicleAction::BOOST:
            result.action_speed = config.enable_boost
                ? std::min(config.max_speed,
                           config.nominal_speed * config.boost_ratio)
                : config.nominal_speed;
            break;
    }
    result.action_speed = std::max(0.0, result.action_speed);
    result.decision_period = std::max(0.0, config.rolling_refresh_period);
    result.braking_time =
        result.action_speed / std::max(1e-6, config.max_decel);
    result.time_margin = std::max(0.0, config.dynamic_stop_time_margin);
    result.stop_threshold = result.decision_period + result.braking_time +
                            result.time_margin;
    result.stop_required =
        vehicle_ttc <= result.stop_threshold + 1e-9;
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
