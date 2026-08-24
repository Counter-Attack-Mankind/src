#pragma once

#include <cstddef>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/path_track.h"

namespace forklift_planner {
namespace multi_vehicle {

// Static geometric prior for one opposing, traversal-local overlap component.
// It intentionally carries no runtime relevance, priority, owner, reservation,
// occupancy, release or action state.
struct SharedSegmentCandidate {
    int id = -1;
    int path_gen_a = -1;
    int path_gen_b = -1;
    int traversal_a = -1;
    int traversal_b = -1;
    WpType direction_a = WpType::FORWARD;
    WpType direction_b = WpType::FORWARD;
    double s_a_enter = 0.0;
    double s_a_exit = 0.0;
    double s_b_enter = 0.0;
    double s_b_exit = 0.0;
    double direction_dot_min = 0.0;
    double direction_dot_max = 0.0;
    double direction_dot_mean = 0.0;
    std::size_t sample_count = 0;
    std::size_t strong_opposing_count = 0;
    double strong_opposing_ratio = 0.0;
    double aabb_min_x = 0.0;
    double aabb_min_y = 0.0;
    double aabb_max_x = 0.0;
    double aabb_max_y = 0.0;
    bool aabb_valid = false;
};

// Scans both complete fixed tracks. Candidate IDs are deterministic for the
// same ordered tracks/path generations and are local to the returned vector.
std::vector<SharedSegmentCandidate> computeSharedSegmentCandidates(
    const PathTrack& track_a, int path_gen_a,
    const PathTrack& track_b, int path_gen_b,
    const MapParam& map_param, const MultiVehicleConfig& config);

}  // namespace multi_vehicle
}  // namespace forklift_planner
