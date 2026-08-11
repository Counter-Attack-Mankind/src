#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/path_track.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

// Shadow-only task-lifecycle prediction types. They deliberately contain no
// holder, waiter, action, priority or reservation state.
enum class FutureSegmentType { MOTION, DWELL, UNKNOWN };
enum class FutureCertainty { COMMITTED, PREVIEW, UNKNOWN };

struct MissionLegId {
    int vehicle_id = -1;
    int expected_path_gen = -1;
    int ordinal = -1;
};

struct FutureMissionSegment {
    int segment_id = -1;
    MissionLegId mission_leg_id;
    MissionPhase phase = MissionPhase::DIRECT_TO_B;
    FutureSegmentType type = FutureSegmentType::UNKNOWN;
    FutureCertainty certainty = FutureCertainty::UNKNOWN;

    // Filled by FutureTrajectoryGenerator. Motion duration is a result of the
    // existing nominal speed/acceleration/curvature model, not a task input.
    double start_time = -1.0;
    double duration = -1.0;

    PathTrack track;
    double start_s = 0.0;
    double end_s = 0.0;
    RoughWp hold_pose{};
};

struct FutureMissionPlan {
    int vehicle_id = -1;
    int source_path_gen = -1;
    double horizon = 0.0;
    double initial_speed = 0.0;
    std::vector<FutureMissionSegment> segments;
};

struct FutureSample {
    double t = 0.0;
    RoughWp pose{};
    double speed = 0.0;
    OBB body;
    MissionPhase phase = MissionPhase::DIRECT_TO_B;
    int segment_id = -1;
    MissionLegId mission_leg_id;
    double path_s = 0.0;
    FutureCertainty certainty = FutureCertainty::UNKNOWN;
};

struct FutureMissionTrajectory {
    FutureMissionPlan plan;
    std::vector<FutureSample> samples;
};

class FutureMissionPlanBuilder {
public:
    explicit FutureMissionPlanBuilder(const MultiVehicleConfig& cfg)
        : cfg_(cfg) {}

    // next_pickup_track is a read-only B->A1 preview for the B slot reached by
    // the current TO_B/UNLOAD_DWELL phase. No task allocation is performed.
    FutureMissionPlan build(
        const VehicleAgent& vehicle, double horizon,
        const std::optional<PathTrack>& next_pickup_track = std::nullopt) const;

private:
    const MultiVehicleConfig& cfg_;
};

class FutureTrajectoryGenerator {
public:
    FutureTrajectoryGenerator(const MapParam& mp,
                              const MultiVehicleConfig& cfg)
        : mp_(mp), cfg_(cfg) {}

    // Resolves segment start_time/duration in the supplied shadow plan and
    // returns one nominal lifecycle trajectory. It never mutates VehicleAgent.
    std::vector<FutureSample> generate(FutureMissionPlan& plan) const;

private:
    double curvatureSpeedAt(const PathTrack& track, double path_s) const;
    double limitedSpeed(double current, double desired, double dt) const;

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
};

const char* futureSegmentTypeName(FutureSegmentType type);
const char* futureCertaintyName(FutureCertainty certainty);

}  // namespace multi_vehicle
}  // namespace forklift_planner
