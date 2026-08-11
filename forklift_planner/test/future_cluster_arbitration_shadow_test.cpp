#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/future_cluster_arbitration_shadow.h"

namespace mv = forklift_planner::multi_vehicle;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

mv::FutureConflictZone zone(int id, double a_enter, double a_exit,
                            double b_enter, double b_exit,
                            int generation_a = 10,
                            int generation_b = 20) {
    mv::FutureConflictZone value;
    value.future_zone_id = id;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.s_a_enter = a_enter;
    value.s_a_exit = a_exit;
    value.s_b_enter = b_enter;
    value.s_b_exit = b_exit;
    value.path_generation_a = generation_a;
    value.path_generation_b = generation_b;
    return value;
}

mv::FutureConflictCluster cluster(
    std::initializer_list<mv::FutureConflictZone> zones) {
    mv::FutureConflictCluster value;
    value.cluster_id = 7;
    value.horizon_snapshot_id = 5536;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    for (const auto& item : zones) {
        value.member_zone_ids.push_back(item.future_zone_id);
        value.member_zones.push_back(item);
    }
    return value;
}

mv::VehicleAgent vehicle(int id, int generation, double path_s) {
    mv::VehicleAgent value;
    value.id = id;
    value.path_gen = generation;
    value.path_s = path_s;
    return value;
}

void test5536MixedZoneHoldersBecomeOneClusterHolder() {
    const auto input = cluster({
        zone(0, 0.800, 3.125, 0.550, 3.050),
        zone(1, 2.375, 2.550, 0.500, 1.025),
    });
    const auto a = vehicle(0, 10, 2.050);  // Inside zone0 only.
    const auto b = vehicle(1, 20, 0.541);  // Inside zone1 only.
    mv::ClusterArbitrationShadowContext context;
    context.priority_winner_id = 0;

    const auto result = mv::FutureClusterArbitrationShadow().evaluate(
        input, a, b, context);
    require(result.member_zone_holders.size() == 2,
            "5536 must retain both member-zone diagnostics");
    require(result.member_zone_holders[0].second == 0 &&
                result.member_zone_holders[1].second == 1,
            "5536 zone-level shadow must expose opposite holders");
    require(result.zone_level_mixed_holders,
            "5536 must flag mixed zone holders");
    require(result.holder_id == 0 && result.waiter_id == 1,
            "cluster arbitration must emit exactly one holder/waiter pair");
    require(result.decision_reason == "actual_occupancy_both_priority",
            "both-inside cluster must use deterministic existing priority");
    require(std::abs(result.stop_boundary_s - 0.500) < 1e-9,
            "waiter boundary must be the earliest entry of every member");
    require(!result.all_members_cleared,
            "occupied 5536 cluster must not be released");
    std::cout << "PASS 5536 mixed zone holders -> one cluster holder\n";
}

void testA1DeparturePrecedence() {
    const auto input = cluster({
        zone(0, 0.000, 0.650, 4.800, 5.425),
        zone(1, 0.800, 1.900, 4.125, 5.125),
    });
    const auto a = vehicle(0, 10, 0.0);
    const auto b = vehicle(1, 20, 0.0);
    mv::ClusterArbitrationShadowContext context;
    context.departure_cluster_owner_id = 0;
    context.future_a1_owner_id = 1;
    context.priority_winner_id = 1;

    const auto result = mv::FutureClusterArbitrationShadow().evaluate(
        input, a, b, context);
    require(result.holder_id == 0 && result.waiter_id == 1,
            "departure commitment must own the whole A1 cluster");
    require(result.decision_reason == "departure_cluster_commitment",
            "departure commitment must precede Future A1 and priority");
    require(std::abs(result.stop_boundary_s - 4.125) < 1e-9,
            "A1 waiter boundary must include every member zone");
    std::cout << "PASS A1 departure cluster precedence\n";
}

void testSingleZoneMatchesPriority() {
    const auto input = cluster({zone(0, 1.0, 2.0, 3.0, 4.0)});
    const auto a = vehicle(0, 10, 0.0);
    const auto b = vehicle(1, 20, 0.0);
    mv::ClusterArbitrationShadowContext context;
    context.priority_winner_id = 1;

    const auto result = mv::FutureClusterArbitrationShadow().evaluate(
        input, a, b, context);
    require(result.holder_id == 1 && result.waiter_id == 0,
            "single-zone cluster must preserve priority winner");
    require(result.decision_reason == "priority_winner",
            "single-zone cluster must report priority fallback");
    require(std::abs(result.stop_boundary_s - 1.0) < 1e-9,
            "single-zone stop boundary must use waiter entry");
    require(!result.zone_level_mixed_holders,
            "single-zone cluster cannot have mixed holders");
    std::cout << "PASS ordinary single-zone priority\n";
}

void testReleaseRequiresEveryMemberCleared() {
    const auto input = cluster({
        zone(0, 0.800, 3.125, 0.550, 3.050),
        zone(1, 2.375, 2.550, 0.500, 1.025),
    });
    mv::ClusterArbitrationShadowContext context;
    context.priority_winner_id = 0;
    const auto b = vehicle(1, 20, 0.0);

    auto partial = mv::FutureClusterArbitrationShadow().evaluate(
        input, vehicle(0, 10, 2.600), b, context);
    require(!partial.all_members_cleared,
            "clearing one member must not release the cluster");

    auto complete = mv::FutureClusterArbitrationShadow().evaluate(
        input, vehicle(0, 10, 3.200), b, context);
    require(complete.all_members_cleared,
            "cluster may release only after all members are cleared");
    require(complete.holder_id == -1 && complete.waiter_id == -1 &&
                complete.decision_reason ==
                    "cluster_released_all_members",
            "released cluster must no longer allocate a holder or waiter");

    auto later_generation = mv::FutureClusterArbitrationShadow().evaluate(
        input, vehicle(0, 11, 0.0), b, context);
    require(later_generation.all_members_cleared,
            "a later path generation means every old member was passed");
    std::cout << "PASS all-member release closure\n";
}

}  // namespace

int main() {
    test5536MixedZoneHoldersBecomeOneClusterHolder();
    testA1DeparturePrecedence();
    testSingleZoneMatchesPriority();
    testReleaseRequiresEveryMemberCleared();
    return 0;
}
