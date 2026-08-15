#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <algorithm>
#include <cmath>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

SpeedCoordinationCandidate evaluateCandidate(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, VehicleAction action_a,
    VehicleAction action_b) {
    SpeedCoordinationCandidate candidate;
    candidate.action_a = action_a;
    candidate.action_b = action_b;
    const auto prediction_a = predictTrajectory(
        vehicle_a, map_param, config, action_a, prediction_horizon);
    const auto prediction_b = predictTrajectory(
        vehicle_b, map_param, config, action_b, prediction_horizon);
    const PairInteractionResult interaction =
        detectPairInteractionFromPredictions(
            vehicle_a, vehicle_b, potential_zones,
            prediction_a, prediction_b);
    candidate.conflict_free = !interaction.event.valid;
    if (interaction.event.valid) {
        candidate.first_conflict_t = interaction.event.first_t;
    }
    return candidate;
}

}  // namespace

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
    result.original_first_conflict_t =
        nominal_baseline.event.first_t;
    if (preferred_winner_id != vehicle_a.id &&
        preferred_winner_id != vehicle_b.id) {
        result.fallback_required = true;
        result.reason = "no_priority_winner";
        return result;
    }

    result.attempted = true;
    const bool a_is_winner = preferred_winner_id == vehicle_a.id;
    for (VehicleAction yielding_action :
         {VehicleAction::YIELD, VehicleAction::CREEP}) {
        const VehicleAction action_a = a_is_winner
            ? VehicleAction::NOMINAL : yielding_action;
        const VehicleAction action_b = a_is_winner
            ? yielding_action : VehicleAction::NOMINAL;
        result.candidates.push_back(evaluateCandidate(
            vehicle_a, vehicle_b, potential_zones, map_param, config,
            prediction_horizon, action_a, action_b));
        if (!result.candidates.back().conflict_free) continue;

        result.solved_by_speed_adjustment = true;
        result.selected_action_a = action_a;
        result.selected_action_b = action_b;
        result.reason = yielding_action == VehicleAction::YIELD
            ? "minimum_intervention_yield"
            : "minimum_intervention_creep";
        return result;
    }

    result.fallback_required = true;
    result.reason = "candidate_search_failed";
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
    const double frozen_prefix =
        std::max(std::max(0.0, decision_dt),
                 std::max(0.0, config.rolling_refresh_period));
    const double required_distance =
        braking_distance + speed * frozen_prefix;
    return available_distance <= required_distance + 1e-9;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
