#pragma once

#include <optional>
#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class DynamicInterventionBand {
    NEAR,
    MID,
    FAR,
};

DynamicInterventionBand classifyDynamicInterventionBand(
    double first_conflict_t, const MultiVehicleConfig& config);

const char* dynamicInterventionBandName(DynamicInterventionBand band);

struct SelectedSpeedActionEvaluation {
    VehicleAction action_a = VehicleAction::NOMINAL;
    VehicleAction action_b = VehicleAction::NOMINAL;
    bool conflict_free = false;
    std::optional<double> first_conflict_t;
    std::optional<double> conflict_delay;
};

struct PairSpeedCoordinationResult {
    bool attempted = false;
    bool action_selected = false;
    bool emergency_stop = false;
    VehicleAction baseline_action_a = VehicleAction::NOMINAL;
    VehicleAction baseline_action_b = VehicleAction::NOMINAL;
    VehicleAction selected_action_a = VehicleAction::NOMINAL;
    VehicleAction selected_action_b = VehicleAction::NOMINAL;
    std::optional<double> original_first_conflict_t;
    SelectedSpeedActionEvaluation evaluation;
    std::string reason;
};

VehicleAction selectRollingSpeedAction(
    DynamicInterventionBand band, bool emergency_stop);

SelectedSpeedActionEvaluation evaluateSelectedAction(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, VehicleAction action_a,
    VehicleAction action_b,
    std::optional<double> original_first_conflict_t = std::nullopt);

// Pure two-vehicle rolling action selection. The priority winner remains
// NOMINAL; the loser receives exactly one target for this rolling period:
// FAR=NOMINAL, MID=YIELD, NEAR=CREEP, or STOP for an explicit emergency.
// The selected action is evaluated once over the full horizon for trend
// diagnostics, but a remaining conflict does not reject that action.
PairSpeedCoordinationResult evaluatePairSpeedCoordination(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, int preferred_winner_id,
    bool emergency_stop = false);

// Conservative boundary for deciding whether a new conflict is still a
// distant speed-shaping problem. It uses the legacy 1 cm stop line plus the
// distance that can be consumed by one frozen rolling-plan prefix.
bool hasInsufficientBrakingMargin(
    const VehicleAgent& vehicle, double conflict_entry_s,
    const MultiVehicleConfig& config, double decision_dt,
    double stop_buffer = 0.01);

}  // namespace multi_vehicle
}  // namespace forklift_planner
