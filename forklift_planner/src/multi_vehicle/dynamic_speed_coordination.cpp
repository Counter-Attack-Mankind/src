#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <algorithm>
#include <cmath>

namespace forklift_planner {
namespace multi_vehicle {

DynamicInterventionBand classifyDynamicInterventionBand(
    double first_conflict_t, const MultiVehicleConfig& config) {
    if (first_conflict_t >= config.dynamic_speed_far_threshold) {
        return DynamicInterventionBand::FAR;
    }
    if (first_conflict_t >= config.dynamic_speed_near_threshold) {
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
    std::optional<double> original_first_conflict_t) {
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
        evaluation.first_conflict_t = interaction.event.first_t;
        if (original_first_conflict_t) {
            evaluation.conflict_delay =
                interaction.event.first_t - *original_first_conflict_t;
        }
    }
    return evaluation;
}

PairSpeedCoordinationResult evaluatePairSpeedCoordination(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, int preferred_winner_id,
    bool emergency_stop) {
    (void)potential_zones;
    (void)map_param;
    (void)prediction_horizon;
    PairSpeedCoordinationResult result;
    if (!nominal_baseline.event.valid) {
        result.reason = "baseline_clear";
        return result;
    }
    result.original_first_conflict_t = nominal_baseline.event.first_t;
    if (preferred_winner_id != vehicle_a.id &&
        preferred_winner_id != vehicle_b.id) {
        result.reason = "no_priority_winner";
        return result;
    }

    result.attempted = true;
    result.action_selected = true;
    result.emergency_stop = emergency_stop;
    result.selected_winner_id = preferred_winner_id;
    const DynamicInterventionBand band = classifyDynamicInterventionBand(
        nominal_baseline.event.first_t, config);
    const bool a_is_priority = preferred_winner_id == vehicle_a.id;
    const VehicleAction yielding_action =
        selectRollingSpeedAction(band, emergency_stop);
    result.selected_action_a = a_is_priority
        ? VehicleAction::NOMINAL : yielding_action;
    result.selected_action_b = a_is_priority
        ? yielding_action : VehicleAction::NOMINAL;

    if (result.emergency_stop) {
        result.reason = "rolling_emergency_stop";
    } else if (band == DynamicInterventionBand::FAR) {
        result.reason = "rolling_far_nominal";
    } else if (band == DynamicInterventionBand::MID) {
        result.reason = "rolling_mid_yield";
    } else {
        result.reason = "rolling_near_creep";
    }
    return result;
}

TtcStopBoundary evaluateTtcStopBoundary(
    double first_conflict_t, VehicleAction planned_action,
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
        first_conflict_t <= result.stop_threshold + 1e-9;
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
