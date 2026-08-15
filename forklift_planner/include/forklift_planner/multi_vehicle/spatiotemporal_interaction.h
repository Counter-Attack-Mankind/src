#pragma once

#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

struct InteractionPoint {
    double x = 0.0;
    double y = 0.0;
};

struct TimedOverlapGeometry {
    double t = 0.0;
    std::vector<InteractionPoint> polygon;
};

// Time-independent, compressed geometry produced from the complete fixed
// paths.  It is a potential-interaction index, not proof that every pair of
// arc-length values inside its rectangle overlaps.
struct PotentialConflictZone {
    double s_self_enter = 0.0;
    double s_other_enter = 0.0;
    double s_self_exit = 0.0;
    double s_other_exit = 0.0;
    double x = 0.0;
    double y = 0.0;
    int raw_index = -1;
    double aabb_min_x = 0.0;
    double aabb_min_y = 0.0;
    double aabb_max_x = 0.0;
    double aabb_max_y = 0.0;
    bool aabb_valid = false;
    bool same_dir = false;
};

struct PredictedKinematicSample {
    double t = 0.0;
    double s = 0.0;
    double speed = 0.0;
    OBB body;
};

struct TimedConflictEvent {
    bool valid = false;
    int associated_zone_index = -1;
    double first_t = 0.0;
    double last_t = 0.0;
    std::vector<TimedOverlapGeometry> timed_overlaps;
};

struct PairInteractionResult {
    int vehicle_a = -1;
    int vehicle_b = -1;
    int path_gen_a = -1;
    int path_gen_b = -1;
    std::vector<PotentialConflictZone> potential_zones;
    TimedConflictEvent event;
};

// Pure baseline prediction used by the current pairwise detector.  It starts
// from current path_s/current_speed and recovers toward NOMINAL while obeying
// the existing acceleration, deceleration, curvature and path-end behavior.
std::vector<PredictedKinematicSample> predictBaselineTrajectory(
    const VehicleAgent& vehicle, const MapParam& map_param,
    const MultiVehicleConfig& config, double prediction_horizon);

// Pure synchronized-OBB detector.  It returns only the first contiguous
// overlap event and associates its first sample with the nearest compressed
// potential zone using the legacy interval-distance rule.
PairInteractionResult detectPairInteractionFromPredictions(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PotentialConflictZone>& potential_zones,
    const std::vector<PredictedKinematicSample>& prediction_a,
    const std::vector<PredictedKinematicSample>& prediction_b);

std::vector<InteractionPoint> intersectObbs(const OBB& a, const OBB& b);

}  // namespace multi_vehicle
}  // namespace forklift_planner
