#include "forklift_planner/multi_vehicle/future_conflict_cluster_shadow.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <set>
#include <utility>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

struct Interval {
    double enter = 0.0;
    double exit = 0.0;
};

bool overlapsOrTouches(const Interval& lhs, const Interval& rhs) {
    return lhs.enter <= rhs.exit + 1e-9 &&
           rhs.enter <= lhs.exit + 1e-9;
}

Interval intervalForVehicle(const FutureConflictZone& zone, int vehicle_id) {
    if (zone.vehicle_a == vehicle_id) {
        return Interval{zone.s_a_enter, zone.s_a_exit};
    }
    return Interval{zone.s_b_enter, zone.s_b_exit};
}

class DisjointSet {
public:
    explicit DisjointSet(std::size_t size) : parent_(size), rank_(size, 0) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    std::size_t find(std::size_t value) {
        if (parent_[value] != value) parent_[value] = find(parent_[value]);
        return parent_[value];
    }

    void unite(std::size_t lhs, std::size_t rhs) {
        lhs = find(lhs);
        rhs = find(rhs);
        if (lhs == rhs) return;
        if (rank_[lhs] < rank_[rhs]) std::swap(lhs, rhs);
        parent_[rhs] = lhs;
        if (rank_[lhs] == rank_[rhs]) ++rank_[lhs];
    }

private:
    std::vector<std::size_t> parent_;
    std::vector<int> rank_;
};

}  // namespace

const char* futureConflictClusterMergeReasonName(
    FutureConflictClusterMergeReason reason) {
    switch (reason) {
        case FutureConflictClusterMergeReason::ARC_OVERLAP_A:
            return "ARC_OVERLAP_A";
        case FutureConflictClusterMergeReason::ARC_OVERLAP_B:
            return "ARC_OVERLAP_B";
    }
    return "UNKNOWN";
}

std::vector<FutureConflictCluster>
FutureConflictClusterShadowBuilder::build(
    const std::vector<FutureConflictZone>& zones,
    std::uint64_t horizon_snapshot_id) const {
    // One invocation is one horizon snapshot.  Never connect zones supplied by
    // different invocations, even when their temporary zone ids happen to
    // match.
    std::map<std::pair<int, int>, std::vector<std::size_t>> pair_groups;
    for (std::size_t index = 0; index < zones.size(); ++index) {
        const FutureConflictZone& zone = zones[index];
        pair_groups[{std::min(zone.vehicle_a, zone.vehicle_b),
                     std::max(zone.vehicle_a, zone.vehicle_b)}]
            .push_back(index);
    }

    std::vector<FutureConflictCluster> result;
    for (const auto& pair_group : pair_groups) {
        const int canonical_a = pair_group.first.first;
        const int canonical_b = pair_group.first.second;
        const std::vector<std::size_t>& members = pair_group.second;
        DisjointSet components(members.size());

        for (std::size_t i = 0; i < members.size(); ++i) {
            const FutureConflictZone& lhs = zones[members[i]];
            for (std::size_t j = i + 1; j < members.size(); ++j) {
                const FutureConflictZone& rhs = zones[members[j]];
                const bool overlap_a = overlapsOrTouches(
                    intervalForVehicle(lhs, canonical_a),
                    intervalForVehicle(rhs, canonical_a));
                const bool overlap_b = overlapsOrTouches(
                    intervalForVehicle(lhs, canonical_b),
                    intervalForVehicle(rhs, canonical_b));
                if (overlap_a || overlap_b) components.unite(i, j);
            }
        }

        std::map<std::size_t, std::vector<std::size_t>> grouped_components;
        for (std::size_t i = 0; i < members.size(); ++i) {
            grouped_components[components.find(i)].push_back(members[i]);
        }

        std::vector<FutureConflictCluster> pair_clusters;
        for (const auto& component : grouped_components) {
            FutureConflictCluster cluster;
            cluster.horizon_snapshot_id = horizon_snapshot_id;
            cluster.vehicle_a = canonical_a;
            cluster.vehicle_b = canonical_b;
            for (std::size_t zone_index : component.second) {
                cluster.member_zones.push_back(zones[zone_index]);
            }
            std::sort(cluster.member_zones.begin(), cluster.member_zones.end(),
                      [](const FutureConflictZone& lhs,
                         const FutureConflictZone& rhs) {
                          return lhs.future_zone_id < rhs.future_zone_id;
                      });
            for (const FutureConflictZone& zone : cluster.member_zones) {
                cluster.member_zone_ids.push_back(zone.future_zone_id);
            }

            std::set<FutureConflictClusterMergeReason> reasons;
            for (std::size_t i = 0; i < cluster.member_zones.size(); ++i) {
                for (std::size_t j = i + 1;
                     j < cluster.member_zones.size(); ++j) {
                    const FutureConflictZone& lhs = cluster.member_zones[i];
                    const FutureConflictZone& rhs = cluster.member_zones[j];
                    if (overlapsOrTouches(
                            intervalForVehicle(lhs, canonical_a),
                            intervalForVehicle(rhs, canonical_a))) {
                        reasons.insert(
                            FutureConflictClusterMergeReason::ARC_OVERLAP_A);
                    }
                    if (overlapsOrTouches(
                            intervalForVehicle(lhs, canonical_b),
                            intervalForVehicle(rhs, canonical_b))) {
                        reasons.insert(
                            FutureConflictClusterMergeReason::ARC_OVERLAP_B);
                    }
                }
            }
            cluster.merge_reasons.assign(reasons.begin(), reasons.end());
            pair_clusters.push_back(std::move(cluster));
        }

        std::sort(pair_clusters.begin(), pair_clusters.end(),
                  [](const FutureConflictCluster& lhs,
                     const FutureConflictCluster& rhs) {
                      return lhs.member_zone_ids.front() <
                             rhs.member_zone_ids.front();
                  });
        for (FutureConflictCluster& cluster : pair_clusters) {
            cluster.cluster_id = static_cast<int>(result.size());
            result.push_back(std::move(cluster));
        }
    }
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
