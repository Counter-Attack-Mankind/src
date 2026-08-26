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
    bool residual_evaluated = false;
    bool residual_conflict = false;
    bool priority_safety_stop = false;
    VehicleAction selected_action_a = VehicleAction::NOMINAL;
    VehicleAction selected_action_b = VehicleAction::NOMINAL;
    int selected_winner_id = -1;
    std::optional<double> original_first_conflict_t;
    std::optional<double> residual_first_conflict_t;
    std::optional<double> priority_stop_threshold;
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

// Pure two-vehicle rolling TTC response. The supplied priority vehicle is not
// a reservation owner. The yielding vehicle first receives FAR=NOMINAL,
// MID=YIELD, NEAR=CREEP, or emergency STOP. One synchronized residual check
// then predicts priority=NOMINAL with that selected yielding action; priority
// stops only if the residual physical conflict is still inside the existing
// NOMINAL TTC stop boundary. The priority order is never swapped.
PairSpeedCoordinationResult evaluatePairSpeedCoordination(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, int preferred_winner_id,
    bool emergency_stop = false);

struct TtcStopBoundary {
    VehicleAction planned_action = VehicleAction::STOP;
    double action_speed = 0.0;
    double decision_period = 0.0;
    double braking_time = 0.0;
    double time_margin = 0.0;
    double stop_threshold = 0.0;
    bool stop_required = false;
};

// Ordinary rolling TTC safety boundary. It uses only the NOMINAL baseline's
// first conflict time and the speed of the action planned for this period; it
// does not re-predict that action or evaluate full-horizon clearance.
TtcStopBoundary evaluateTtcStopBoundary(
    double first_conflict_t, VehicleAction planned_action,
    const MultiVehicleConfig& config);

}  // namespace multi_vehicle
}  // namespace forklift_planner
