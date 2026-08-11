#pragma once

#include <cstddef>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/future_conflict_zone_shadow.h"
#include "forklift_planner/multi_vehicle/future_mission_trajectory.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/prediction_shadow_comparator.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

struct ShadowConflictZone {
    int vehicle_a = -1;
    int vehicle_b = -1;
    int zone_index = -1;
    double s_a_enter = 0.0;
    double s_a_exit = 0.0;
    double s_b_enter = 0.0;
    double s_b_exit = 0.0;
};

struct ShadowTimedEvent {
    bool valid = false;
    int vehicle_a = -1;
    int vehicle_b = -1;
    double first_t = 0.0;
    double last_t = 0.0;
    double overlap_duration = 0.0;
    std::size_t overlap_samples = 0;
    int matched_zone = -1;
    int future_zone_id = -1;
    int segment_a = -1;
    int segment_b = -1;
    FutureCertainty certainty_a = FutureCertainty::UNKNOWN;
    FutureCertainty certainty_b = FutureCertainty::UNKNOWN;
    MissionPhase phase_a = MissionPhase::DIRECT_TO_B;
    MissionPhase phase_b = MissionPhase::DIRECT_TO_B;
    double x = 0.0;
    double y = 0.0;
};

enum class TimedConflictShadowClass {
    MATCH,
    NEW_FUTURE_LIFECYCLE_CONFLICT,
    PREDICTION_ERROR,
    ZONE_MAPPING_DIFFERENCE
};

struct TimedConflictShadowReport {
    int vehicle_a = -1;
    int vehicle_b = -1;
    ShadowTimedEvent old_event;
    ShadowTimedEvent new_event;
    TimedConflictShadowClass classification =
        TimedConflictShadowClass::MATCH;
};

class TimedConflictShadowChecker {
public:
    TimedConflictShadowChecker(const MapParam& mp,
                               const MultiVehicleConfig& cfg)
        : mp_(mp), cfg_(cfg) {}

    TimedConflictShadowReport compare(
        const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
        const std::vector<LegacyPredictionSample>& legacy_a,
        const std::vector<LegacyPredictionSample>& legacy_b,
        const FutureMissionTrajectory& future_a,
        const FutureMissionTrajectory& future_b,
        const std::vector<ShadowConflictZone>& zones,
        const std::vector<FutureConflictZone>& future_zones = {}) const;

private:
    ShadowTimedEvent checkLegacy(
        int vehicle_a, int vehicle_b,
        const std::vector<LegacyPredictionSample>& samples_a,
        const std::vector<LegacyPredictionSample>& samples_b,
        const std::vector<ShadowConflictZone>& zones) const;
    ShadowTimedEvent checkFuture(
        const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
        const FutureMissionTrajectory& trajectory_a,
        const FutureMissionTrajectory& trajectory_b,
        const std::vector<ShadowConflictZone>& zones,
        const std::vector<FutureConflictZone>& future_zones) const;

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
};

const char* timedConflictShadowClassName(TimedConflictShadowClass value);

}  // namespace multi_vehicle
}  // namespace forklift_planner
