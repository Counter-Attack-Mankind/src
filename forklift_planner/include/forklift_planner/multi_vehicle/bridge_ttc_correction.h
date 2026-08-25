#pragma once

#include <limits>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class BridgeBacktrackEndReason {
    NOT_EVALUATED,
    RELATION_DISTANCE_LOST,
    RELATION_DIRECTION_LOST,
    PATH_START,
    INVALID_GEOMETRY,
};

const char* bridgeBacktrackEndReasonName(BridgeBacktrackEndReason reason);

struct VehicleBridgeTtcCorrection {
    bool bridge_related = false;
    double collision_s = 0.0;
    double matched_other_s = 0.0;
    double near_boundary_s = 0.0;
    double original_ttc = std::numeric_limits<double>::infinity();
    double corrected_ttc = std::numeric_limits<double>::infinity();
    double collision_direction_dot = 1.0;
    double collision_match_distance =
        std::numeric_limits<double>::infinity();
    WpType collision_type = WpType::FORWARD;
    WpType boundary_type = WpType::FORWARD;
    int self_traversal_changes = 0;
    int nearest_other_traversal_changes = 0;
    double end_query_s = 0.0;
    double end_matched_other_s = 0.0;
    double end_match_distance = std::numeric_limits<double>::infinity();
    double end_direction_dot = 1.0;
    BridgeBacktrackEndReason backtrack_end_reason =
        BridgeBacktrackEndReason::NOT_EVALUATED;
    int backtrack_samples = 0;
    int nearest_search_evaluations = 0;
};

struct PairBridgeTtcCorrection {
    bool baseline_conflict = false;
    VehicleBridgeTtcCorrection a;
    VehicleBridgeTtcCorrection b;
};

// Evaluates only a baseline conflict that already exists in the synchronized
// prediction. It performs no full path-pair matrix scan and owns no state.
PairBridgeTtcCorrection evaluateBridgeTtcCorrection(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PredictedKinematicSample>& prediction_a,
    const std::vector<PredictedKinematicSample>& prediction_b,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config);

}  // namespace multi_vehicle
}  // namespace forklift_planner
