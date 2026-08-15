#pragma once

#include <optional>
#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"

namespace forklift_planner {
namespace multi_vehicle {

struct SpeedCoordinationCandidate {
    VehicleAction action_a = VehicleAction::NOMINAL;
    VehicleAction action_b = VehicleAction::NOMINAL;
    bool conflict_free = false;
    std::optional<double> first_conflict_t;
};

struct PairSpeedCoordinationResult {
    bool attempted = false;
    bool solved_by_speed_adjustment = false;
    bool fallback_required = false;
    VehicleAction baseline_action_a = VehicleAction::NOMINAL;
    VehicleAction baseline_action_b = VehicleAction::NOMINAL;
    VehicleAction selected_action_a = VehicleAction::NOMINAL;
    VehicleAction selected_action_b = VehicleAction::NOMINAL;
    std::optional<double> original_first_conflict_t;
    std::vector<SpeedCoordinationCandidate> candidates;
    std::string reason;
};

// Pure two-vehicle counterfactual search. The existing priority winner stays
// NOMINAL while the other vehicle is tried at YIELD and then CREEP. A
// candidate succeeds only if no synchronized OBB event remains anywhere in
// the supplied horizon.
PairSpeedCoordinationResult evaluatePairSpeedCoordination(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config,
    double prediction_horizon, int preferred_winner_id);

// Conservative boundary for deciding whether a new conflict is still a
// distant speed-shaping problem. It uses the legacy 1 cm stop line plus the
// distance that can be consumed by one frozen rolling-plan prefix.
bool hasInsufficientBrakingMargin(
    const VehicleAgent& vehicle, double conflict_entry_s,
    const MultiVehicleConfig& config, double decision_dt,
    double stop_buffer = 0.01);

}  // namespace multi_vehicle
}  // namespace forklift_planner
