#pragma once

#include <cstddef>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/future_mission_trajectory.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

// Diagnostic mirror of the current-track predictor local to
// RuleEngine::resolvePairwiseConflicts(). It is deliberately not consumed by
// RuleEngine and cannot affect holder/waiter/action decisions.
struct LegacyPredictionSample {
    double t = 0.0;
    double s = 0.0;
    double speed = 0.0;
    RoughWp pose{};
    OBB body;
};

class LegacyPredictionShadowGenerator {
public:
    LegacyPredictionShadowGenerator(const MapParam& mp,
                                    const MultiVehicleConfig& cfg)
        : mp_(mp), cfg_(cfg) {}

    std::vector<LegacyPredictionSample> generate(
        const VehicleAgent& vehicle, double horizon) const;

private:
    double curvatureSpeedAt(const PathTrack& track, double path_s) const;

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
};

enum class ShadowMismatchKind {
    NONE,
    FUTURE_LIFECYCLE_SEGMENT,
    OLD_PREDICTION_UNAVAILABLE,
    TIME_GRID
};

struct PredictionShadowPoint {
    bool valid = false;
    double t = 0.0;
    double old_s = 0.0;
    double new_s = 0.0;
    double position_error = 0.0;
    MissionPhase new_phase = MissionPhase::DIRECT_TO_B;
    int new_segment_id = -1;
    ShadowMismatchKind mismatch = ShadowMismatchKind::NONE;
};

struct PredictionShadowReport {
    int vehicle_id = -1;
    std::size_t old_sample_count = 0;
    std::size_t new_sample_count = 0;
    std::size_t matched_sample_count = 0;
    std::size_t segment_mismatch_count = 0;
    std::size_t time_mismatch_count = 0;

    double max_s_error = 0.0;
    double mean_s_error = 0.0;
    double max_position_error = 0.0;
    double mean_position_error = 0.0;
    double max_speed_error = 0.0;
    double max_body_center_error = 0.0;
    double max_body_yaw_error = 0.0;

    PredictionShadowPoint maximum_error;
    PredictionShadowPoint first_mismatch;
};

class PredictionShadowComparator {
public:
    PredictionShadowReport compare(
        const VehicleAgent& vehicle,
        const std::vector<LegacyPredictionSample>& legacy,
        const FutureMissionTrajectory& future) const;
};

const char* shadowMismatchKindName(ShadowMismatchKind kind);

}  // namespace multi_vehicle
}  // namespace forklift_planner
