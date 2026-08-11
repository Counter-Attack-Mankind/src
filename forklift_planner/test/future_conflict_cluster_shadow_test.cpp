#include "forklift_planner/multi_vehicle/future_conflict_cluster_shadow.h"

#include <algorithm>
#include <iostream>
#include <vector>

using forklift_planner::multi_vehicle::FutureConflictCluster;
using forklift_planner::multi_vehicle::FutureConflictClusterMergeReason;
using forklift_planner::multi_vehicle::FutureConflictClusterShadowBuilder;
using forklift_planner::multi_vehicle::FutureConflictZone;

namespace {

int fail(const char* message) {
    std::cerr << "future_conflict_cluster_shadow_test: " << message << '\n';
    return 1;
}

FutureConflictZone zone(int id, double a_enter, double a_exit,
                        double b_enter, double b_exit) {
    FutureConflictZone value;
    value.future_zone_id = id;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.segment_id_a = 0;
    value.segment_id_b = 0;
    value.path_generation_a = 10;
    value.path_generation_b = 20;
    value.s_a_enter = a_enter;
    value.s_a_exit = a_exit;
    value.s_b_enter = b_enter;
    value.s_b_exit = b_exit;
    value.x = static_cast<double>(id);
    return value;
}

bool hasReason(const FutureConflictCluster& cluster,
               FutureConflictClusterMergeReason reason) {
    return std::find(cluster.merge_reasons.begin(),
                     cluster.merge_reasons.end(), reason) !=
           cluster.merge_reasons.end();
}

}  // namespace

int main() {
    FutureConflictClusterShadowBuilder builder;

    // 1. A single ordinary crossing remains a one-zone component.
    const auto single = builder.build({zone(0, 1.0, 1.5, 2.0, 2.5)}, 1);
    if (single.size() != 1 || single[0].member_zone_ids !=
                                  std::vector<int>{0} ||
        !single[0].merge_reasons.empty()) {
        return fail("single crossing was not a singleton cluster");
    }

    // 2. The diagnosed 5536 s geometry overlaps on both vehicle arcs.
    const auto deadlock = builder.build(
        {zone(0, 0.800, 3.125, 0.550, 3.050),
         zone(1, 2.375, 2.550, 0.500, 1.025)},
        55361);
    if (deadlock.size() != 1 ||
        deadlock[0].member_zone_ids != std::vector<int>({0, 1}) ||
        !hasReason(deadlock[0],
                   FutureConflictClusterMergeReason::ARC_OVERLAP_A) ||
        !hasReason(deadlock[0],
                   FutureConflictClusterMergeReason::ARC_OVERLAP_B)) {
        return fail("5536 s zone0/zone1 closure was not recognized");
    }

    // 3. A1 departure: disjoint owner arcs still form one component because
    // the other vehicle's conflict intervals overlap.
    const auto departure = builder.build(
        {zone(0, 0.000, 0.650, 4.800, 5.425),
         zone(1, 0.800, 1.900, 4.125, 5.125)},
        2);
    if (departure.size() != 1 ||
        departure[0].member_zone_ids != std::vector<int>({0, 1}) ||
        hasReason(departure[0],
                  FutureConflictClusterMergeReason::ARC_OVERLAP_A) ||
        !hasReason(departure[0],
                   FutureConflictClusterMergeReason::ARC_OVERLAP_B)) {
        return fail("A1 departure other-arc closure was not recognized");
    }

    // 4. Same pair, phase and horizon are insufficient without arc overlap.
    const auto disconnected = builder.build(
        {zone(0, 0.0, 0.5, 2.0, 2.5),
         zone(1, 2.0, 2.5, 8.0, 8.5)},
        3);
    if (disconnected.size() != 2 ||
        disconnected[0].member_zone_ids.size() != 1 ||
        disconnected[1].member_zone_ids.size() != 1) {
        return fail("disconnected zones were incorrectly merged");
    }

    // 5. Closure must be transitive, not only pairwise against the seed.
    const auto transitive = builder.build(
        {zone(0, 0.0, 0.5, 5.0, 6.0),
         zone(1, 0.8, 1.2, 4.0, 5.2),
         zone(2, 1.5, 2.0, 3.0, 4.1)},
        4);
    if (transitive.size() != 1 ||
        transitive[0].member_zone_ids != std::vector<int>({0, 1, 2})) {
        return fail("transitive arc closure failed");
    }

    std::cout << "single_crossing_cluster=[zone0] PASS\n"
              << "5536_cluster=[zone0,zone1] "
                 "reason=ARC_OVERLAP_A/B PASS\n"
              << "a1_departure_cluster=[zone0,zone1] "
                 "reason=ARC_OVERLAP_B PASS\n"
              << "disconnected_same_pair_not_merged=PASS\n"
              << "transitive_closure=PASS\n"
              << "future_conflict_cluster_shadow_test: PASS\n";
    return 0;
}
