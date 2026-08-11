#pragma once

#include <cstdint>
#include <vector>

#include "forklift_planner/multi_vehicle/future_conflict_zone_shadow.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class FutureConflictClusterMergeReason {
    ARC_OVERLAP_A,
    ARC_OVERLAP_B
};

// Shadow-only connected component of FutureConflictZones from one horizon
// snapshot.  It deliberately carries no holder, waiter, reservation or action.
struct FutureConflictCluster {
    int cluster_id = -1;
    std::uint64_t horizon_snapshot_id = 0;
    int vehicle_a = -1;
    int vehicle_b = -1;
    std::vector<int> member_zone_ids;
    std::vector<FutureConflictZone> member_zones;
    std::vector<FutureConflictClusterMergeReason> merge_reasons;
};

class FutureConflictClusterShadowBuilder {
public:
    std::vector<FutureConflictCluster> build(
        const std::vector<FutureConflictZone>& zones,
        std::uint64_t horizon_snapshot_id) const;
};

const char* futureConflictClusterMergeReasonName(
    FutureConflictClusterMergeReason reason);

}  // namespace multi_vehicle
}  // namespace forklift_planner
