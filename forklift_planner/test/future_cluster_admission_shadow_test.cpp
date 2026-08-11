#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "forklift_planner/multi_vehicle/future_cluster_admission_shadow.h"

namespace mv = forklift_planner::multi_vehicle;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

mv::FutureConflictZone zone(int id, double ae, double ax,
                            double be, double bx) {
    mv::FutureConflictZone value;
    value.future_zone_id = id;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.s_a_enter = ae;
    value.s_a_exit = ax;
    value.s_b_enter = be;
    value.s_b_exit = bx;
    value.path_generation_a = 10;
    value.path_generation_b = 20;
    return value;
}

mv::FutureConflictCluster cluster() {
    mv::FutureConflictCluster value;
    value.cluster_id = 1;
    value.horizon_snapshot_id = 2970;
    value.vehicle_a = 0;
    value.vehicle_b = 1;
    value.member_zones = {
        zone(2, 0.800, 3.125, 0.550, 3.050),
        zone(3, 2.375, 2.550, 0.500, 1.025),
    };
    value.member_zone_ids = {2, 3};
    return value;
}

mv::VehicleAgent vehicle(int id, int generation, double path_s) {
    mv::VehicleAgent value;
    value.id = id;
    value.path_gen = generation;
    value.path_s = path_s;
    return value;
}

void testPreEntryAdmissionUsesWholeClusterBoundary() {
    const auto input = cluster();
    const auto a = vehicle(0, 10, 0.2);
    const auto b = vehicle(1, 20, 0.1);
    mv::ClusterArbitrationShadowContext context;
    context.priority_winner_id = 0;
    const auto arbitration = mv::FutureClusterArbitrationShadow().evaluate(
        input, a, b, context);
    const auto admission = mv::FutureClusterAdmissionShadow().evaluate(
        input, arbitration, a, b, 0.01);

    require(std::abs(admission.cluster_enter_s_a - 0.800) < 1e-9,
            "A entry must be the minimum across all members");
    require(std::abs(admission.cluster_enter_s_b - 0.500) < 1e-9,
            "B entry must be the minimum across all members");
    require(std::abs(admission.waiter_stop_s - 0.490) < 1e-9,
            "waiter stop must be upstream of the whole-cluster entry");
    require(admission.admission_valid && admission.prevent_zone_mixing,
            "pre-entry admission must be valid and prevent mixing");
    std::cout << "PASS whole-cluster pre-entry admission\n";
}

void test5536MixedOccupancyIsCounterfactuallyPreventable() {
    const auto input = cluster();
    const auto a = vehicle(0, 10, 2.05);
    const auto b = vehicle(1, 20, 0.541);
    mv::ClusterArbitrationShadowContext context;
    context.priority_winner_id = 0;
    const auto arbitration = mv::FutureClusterArbitrationShadow().evaluate(
        input, a, b, context);
    const auto admission = mv::FutureClusterAdmissionShadow().evaluate(
        input, arbitration, a, b, 0.01);

    require(arbitration.zone_level_mixed_holders,
            "5536 setup must expose mixed member holders");
    require(!admission.admission_valid,
            "admission cannot be enforced after both vehicles entered");
    require(admission.prevent_zone_mixing &&
                admission.zone_mixing_observed,
            "5536 must identify counterfactual prevention capability");
    require(admission.decision_reason ==
                "counterfactual_pre_entry_admission_would_prevent_mixing",
            "5536 reason must distinguish counterfactual from live admission");
    std::cout << "PASS 5536 counterfactual prevention\n";
}

void testReleasedClusterHasNoAdmission() {
    const auto input = cluster();
    const auto a = vehicle(0, 10, 3.2);
    const auto b = vehicle(1, 20, 3.1);
    mv::ClusterArbitrationShadowContext context;
    context.priority_winner_id = 0;
    const auto arbitration = mv::FutureClusterArbitrationShadow().evaluate(
        input, a, b, context);
    const auto admission = mv::FutureClusterAdmissionShadow().evaluate(
        input, arbitration, a, b, 0.01);
    require(arbitration.all_members_cleared,
            "test cluster must be released");
    require(!admission.admission_valid &&
                !admission.prevent_zone_mixing,
            "released cluster must not admit or block either vehicle");
    std::cout << "PASS released cluster admission cleanup\n";
}

void testShadowAdmissionLocksHolderAcrossRefreshes() {
    const auto input = cluster();
    const auto a = vehicle(0, 10, 0.2);
    const auto b = vehicle(1, 20, 0.1);
    mv::FutureClusterAdmissionShadow evaluator;
    mv::FutureClusterAdmissionShadowTracker tracker;

    mv::ClusterArbitrationShadowContext first_context;
    first_context.priority_winner_id = 1;
    auto first_arbitration = mv::FutureClusterArbitrationShadow().evaluate(
        input, a, b, first_context);
    auto first = evaluator.evaluate(input, first_arbitration, a, b, 0.01);
    first = tracker.update(first, first_arbitration, a, b, 100.0);
    require(first.shadow_lock_active && first.holder_id == 1,
            "first valid admission must create a shadow holder lock");

    mv::ClusterArbitrationShadowContext changed_context;
    changed_context.priority_winner_id = 0;
    auto changed_arbitration =
        mv::FutureClusterArbitrationShadow().evaluate(
            input, a, b, changed_context);
    auto changed = evaluator.evaluate(
        input, changed_arbitration, a, b, 0.01);
    changed = tracker.update(changed, changed_arbitration, a, b, 102.0);
    require(changed.holder_id == 1 && changed.waiter_id == 0 &&
                changed.holder_change_suppressed,
            "refresh priority changes must not swap an admitted holder");
    require(changed.decision_reason ==
                "shadow_admission_hold_suppressed_change",
            "suppressed refresh must be explicit in diagnostics");
    std::cout << "PASS horizon-to-horizon admission holder lock\n";
}

}  // namespace

int main() {
    testPreEntryAdmissionUsesWholeClusterBoundary();
    test5536MixedOccupancyIsCounterfactuallyPreventable();
    testReleasedClusterHasNoAdmission();
    testShadowAdmissionLocksHolderAcrossRefreshes();
    return 0;
}
