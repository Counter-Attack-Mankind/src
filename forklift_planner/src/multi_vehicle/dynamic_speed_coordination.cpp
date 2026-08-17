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
    const bool a_is_winner = preferred_winner_id == vehicle_a.id;
    const DynamicInterventionBand band = classifyDynamicInterventionBand(
        nominal_baseline.event.first_t, config);
    const VehicleAction loser_action = selectRollingSpeedAction(
        band, emergency_stop);
    result.selected_action_a = a_is_winner
        ? VehicleAction::NOMINAL : loser_action;
    result.selected_action_b = a_is_winner
        ? loser_action : VehicleAction::NOMINAL;
    result.evaluation = evaluateSelectedAction(
        vehicle_a, vehicle_b, potential_zones, map_param, config,
        prediction_horizon, result.selected_action_a,
        result.selected_action_b, result.original_first_conflict_t);

    if (emergency_stop) {
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

bool hasInsufficientBrakingMargin(
    const VehicleAgent& vehicle, double conflict_entry_s,
    const MultiVehicleConfig& config, double decision_dt,
    double stop_buffer) {
    const double stop_s = std::max(
        0.0, conflict_entry_s - std::max(0.0, stop_buffer) -
                 std::max(0.0, config.safety_margin));
    const double available_distance = stop_s - vehicle.path_s;
    const double speed = std::max(0.0, vehicle.current_speed);
    const double braking_distance =
        speed * speed / (2.0 * std::max(1e-6, config.max_decel));
    const double rolling_prefix =
        std::max(std::max(0.0, decision_dt),
                 std::max(0.0, config.rolling_refresh_period));
    const double required_distance =
        braking_distance + speed * rolling_prefix;
    return available_distance <= required_distance + 1e-9;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
