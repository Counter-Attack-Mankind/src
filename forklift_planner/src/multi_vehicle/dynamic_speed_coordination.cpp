#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"

#include <algorithm>
#include <cmath>
#include <utility>

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
    std::optional<double> original_first_conflict_t,
    const PairInteractionResult* interaction_context,
    int preferred_winner_id) {
    SelectedSpeedActionEvaluation evaluation;
    evaluation.action_a = action_a;
    evaluation.action_b = action_b;
    const auto prediction_a = predictTrajectory(
        vehicle_a, map_param, config, action_a, prediction_horizon);
    const auto prediction_b = predictTrajectory(
        vehicle_b, map_param, config, action_b, prediction_horizon);
    PairInteractionResult interaction;
    if (interaction_context != nullptr &&
        interaction_context->type == PairInteractionType::OPPOSING &&
        interaction_context->shared_segment.valid) {
        const double clearance_time = std::max(
            config.prediction_step,
            config.conflict_margin /
                std::max(1e-6, config.nominal_speed));
        interaction = detectSharedSegmentInteraction(
            vehicle_a, vehicle_b, interaction_context->shared_segment,
            prediction_a, prediction_b, clearance_time,
            preferred_winner_id);
    } else {
        interaction = detectPairInteractionFromPredictions(
            vehicle_a, vehicle_b, potential_zones,
            prediction_a, prediction_b);
    }
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
    result.selected_winner_id = preferred_winner_id;
    const DynamicInterventionBand band = classifyDynamicInterventionBand(
        nominal_baseline.event.first_t, config);
    auto evaluateOrder = [&](int winner_id, bool force_stop) {
        PairSpeedCoordinationResult candidate;
        candidate.attempted = true;
        candidate.action_selected = true;
        candidate.emergency_stop = force_stop;
        candidate.original_first_conflict_t = result.original_first_conflict_t;
        candidate.selected_winner_id = winner_id;
        const bool a_is_winner = winner_id == vehicle_a.id;
        const VehicleAction initial = selectRollingSpeedAction(band, force_stop);
        const std::vector<VehicleAction> attempts =
            force_stop || band == DynamicInterventionBand::FAR
                ? std::vector<VehicleAction>{initial}
                : initial == VehicleAction::YIELD
                    ? std::vector<VehicleAction>{VehicleAction::YIELD,
                                                 VehicleAction::CREEP,
                                                 VehicleAction::STOP}
                    : std::vector<VehicleAction>{VehicleAction::CREEP,
                                                 VehicleAction::STOP};
        for (VehicleAction loser_action : attempts) {
            candidate.selected_action_a = a_is_winner
                ? VehicleAction::NOMINAL : loser_action;
            candidate.selected_action_b = a_is_winner
                ? loser_action : VehicleAction::NOMINAL;
            candidate.evaluation = evaluateSelectedAction(
                vehicle_a, vehicle_b, potential_zones, map_param, config,
                prediction_horizon, candidate.selected_action_a,
                candidate.selected_action_b,
                candidate.original_first_conflict_t,
                &nominal_baseline, winner_id);
            if (candidate.evaluation.conflict_free) break;
        }
        return candidate;
    };

    result = evaluateOrder(preferred_winner_id, emergency_stop);

    // Crossing priority is a preference, not permission to execute a
    // full-horizon candidate that still collides. Winner selection remains in
    // this coordinator: try the opposite order, then STOP/STOP for one rolling
    // period. Opposing occupancy and same-direction longitudinal order are not
    // reversible here.
    if (band != DynamicInterventionBand::FAR &&
        nominal_baseline.type == PairInteractionType::CROSSING &&
        !result.evaluation.conflict_free) {
        const int alternate_winner = preferred_winner_id == vehicle_a.id
            ? vehicle_b.id : vehicle_a.id;
        PairSpeedCoordinationResult alternate =
            evaluateOrder(alternate_winner, false);
        if (alternate.evaluation.conflict_free) {
            result = std::move(alternate);
            result.reason = "rolling_crossing_order_swap";
        } else {
            const SelectedSpeedActionEvaluation both_stop =
                evaluateSelectedAction(
                    vehicle_a, vehicle_b, potential_zones, map_param, config,
                    prediction_horizon, VehicleAction::STOP,
                    VehicleAction::STOP, result.original_first_conflict_t,
                    &nominal_baseline, -1);
            if (both_stop.conflict_free) {
                result.selected_action_a = VehicleAction::STOP;
                result.selected_action_b = VehicleAction::STOP;
                result.selected_winner_id = -1;
                result.emergency_stop = true;
                result.evaluation = both_stop;
                result.reason = "rolling_crossing_both_stop";
            }
        }
    }

    // A candidate that still contains a current/immediate conflict is not a
    // successfully coordinated motion command. Keep the hard guard intact,
    // but do not let either selected "winner" continue NOMINAL while the
    // re-prediction is already unsafe at the current sample.
    if (nominal_baseline.type != PairInteractionType::SAME_DIRECTION &&
        !result.evaluation.conflict_free &&
        result.evaluation.first_conflict_t &&
        *result.evaluation.first_conflict_t <=
            config.prediction_step + 1e-9) {
        result.selected_action_a = VehicleAction::STOP;
        result.selected_action_b = VehicleAction::STOP;
        result.selected_winner_id = -1;
        result.emergency_stop = true;
        result.evaluation = evaluateSelectedAction(
            vehicle_a, vehicle_b, potential_zones, map_param, config,
            prediction_horizon, VehicleAction::STOP, VehicleAction::STOP,
            result.original_first_conflict_t, &nominal_baseline, -1);
        result.reason = "rolling_unresolved_immediate_stop";
    }

    if (!result.reason.empty()) return result;

    const VehicleAction loser_action =
        result.selected_winner_id == vehicle_a.id
            ? result.selected_action_b : result.selected_action_a;
    if (result.emergency_stop) {
        result.reason = "rolling_emergency_stop";
    } else if (loser_action == VehicleAction::STOP) {
        result.reason = nominal_baseline.type == PairInteractionType::OPPOSING
            ? "rolling_opposing_stop" : "rolling_conflict_stop";
    } else if (nominal_baseline.type == PairInteractionType::OPPOSING &&
               loser_action == VehicleAction::CREEP &&
               band == DynamicInterventionBand::MID) {
        result.reason = "rolling_opposing_creep";
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
