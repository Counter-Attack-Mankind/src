#pragma once

#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/future_mission_trajectory.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class FutureConflictZoneSource { CURRENT_TRACK, FUTURE_SEGMENT };

struct FutureConflictZone {
    int future_zone_id = -1;
    int vehicle_a = -1;
    int vehicle_b = -1;
    int segment_id_a = -1;
    int segment_id_b = -1;
    MissionPhase phase_a = MissionPhase::DIRECT_TO_B;
    MissionPhase phase_b = MissionPhase::DIRECT_TO_B;
    FutureCertainty certainty_a = FutureCertainty::UNKNOWN;
    FutureCertainty certainty_b = FutureCertainty::UNKNOWN;
    double s_a_enter = 0.0;
    double s_a_exit = 0.0;
    double s_b_enter = 0.0;
    double s_b_exit = 0.0;
    double x = 0.0;
    double y = 0.0;
    int path_generation_a = -1;
    int path_generation_b = -1;
    FutureConflictZoneSource source =
        FutureConflictZoneSource::FUTURE_SEGMENT;
};

class FutureConflictZoneShadowBuilder {
public:
    FutureConflictZoneShadowBuilder(const MapParam& mp,
                                    const MultiVehicleConfig& cfg)
        : mp_(mp), cfg_(cfg) {}

    std::vector<FutureConflictZone> build(
        const std::vector<FutureMissionTrajectory>& trajectories) const;

private:
    using CacheKey = std::tuple<int, int, int, int, int, int,
                                std::uint64_t, std::uint64_t>;

    std::vector<FutureConflictZone> compute(
        int vehicle_a, int vehicle_b,
        const FutureMissionSegment& segment_a,
        const FutureMissionSegment& segment_b,
        FutureConflictZoneSource source) const;
    std::uint64_t trackSignature(const PathTrack& track) const;

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
    mutable std::map<CacheKey, std::vector<FutureConflictZone>> cache_;
};

const char* futureConflictZoneSourceName(FutureConflictZoneSource source);

}  // namespace multi_vehicle
}  // namespace forklift_planner
