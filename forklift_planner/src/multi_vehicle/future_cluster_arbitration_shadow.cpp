#include "forklift_planner/multi_vehicle/future_cluster_arbitration_shadow.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

struct OrientedZone {
    double enter_a = 0.0;
    double exit_a = 0.0;
    double enter_b = 0.0;
    double exit_b = 0.0;
    int generation_a = -1;
    int generation_b = -1;
};

OrientedZone orient(const FutureConflictZone& zone, int vehicle_a) {
    if (zone.vehicle_a == vehicle_a) {
        return OrientedZone{zone.s_a_enter, zone.s_a_exit,
                            zone.s_b_enter, zone.s_b_exit,
                            zone.path_generation_a,
                            zone.path_generation_b};
    }
    return OrientedZone{zone.s_b_enter, zone.s_b_exit,
                        zone.s_a_enter, zone.s_a_exit,
                        zone.path_generation_b,
                        zone.path_generation_a};
}

bool inside(const VehicleAgent& vehicle, int expected_generation,
            double enter_s, double exit_s) {
    return vehicle.path_gen == expected_generation &&
           vehicle.path_s > enter_s + 1e-9 &&
           vehicle.path_s <= exit_s + 1e-9;
}

bool passed(const VehicleAgent& vehicle, int expected_generation,
            double exit_s) {
    return vehicle.path_gen > expected_generation ||
           (vehicle.path_gen == expected_generation &&
            vehicle.path_s > exit_s + 1e-9);
}

std::pair<int, std::string> selectHolder(
    bool a_inside, bool b_inside, int vehicle_a, int vehicle_b,
    const ClusterArbitrationShadowContext& context) {
    if (a_inside != b_inside) {
        return {a_inside ? vehicle_a : vehicle_b, "actual_occupancy"};
    }
    if (a_inside && b_inside) {
        return {context.priority_winner_id,
                "actual_occupancy_both_priority"};
    }
    if (context.departure_cluster_owner_id >= 0) {
        return {context.departure_cluster_owner_id,
                "departure_cluster_commitment"};
    }
    if (context.future_a1_owner_id >= 0) {
        return {context.future_a1_owner_id, "future_a1_owner"};
    }
    if (context.a_a1_departure != context.b_a1_departure) {
        return {context.a_a1_departure ? vehicle_a : vehicle_b,
                "a1_departure_committed"};
    }
    if (context.a_terminal_docking != context.b_terminal_docking) {
        return {context.a_terminal_docking ? vehicle_a : vehicle_b,
                "terminal_docking"};
    }
    return {context.priority_winner_id, "priority_winner"};
}

}  // namespace

ClusterReservationShadow FutureClusterArbitrationShadow::evaluate(
    const FutureConflictCluster& cluster, const VehicleAgent& vehicle_a,
    const VehicleAgent& vehicle_b,
    const ClusterArbitrationShadowContext& context) const {
    ClusterReservationShadow result;
    result.cluster_id = cluster.cluster_id;
    result.horizon_snapshot_id = cluster.horizon_snapshot_id;
    result.vehicle_a = vehicle_a.id;
    result.vehicle_b = vehicle_b.id;
    result.member_zone_ids = cluster.member_zone_ids;

    double first_a = std::numeric_limits<double>::infinity();
    double first_b = std::numeric_limits<double>::infinity();
    double last_a = 0.0;
    double last_b = 0.0;
    bool all_cleared = !cluster.member_zones.empty();

    for (const FutureConflictZone& zone : cluster.member_zones) {
        const OrientedZone value = orient(zone, vehicle_a.id);
        first_a = std::min(first_a, value.enter_a);
        first_b = std::min(first_b, value.enter_b);
        last_a = std::max(last_a, value.exit_a);
        last_b = std::max(last_b, value.exit_b);
        const bool inside_a = inside(vehicle_a, value.generation_a,
                                     value.enter_a, value.exit_a);
        const bool inside_b = inside(vehicle_b, value.generation_b,
                                     value.enter_b, value.exit_b);
        result.vehicle_a_inside = result.vehicle_a_inside || inside_a;
        result.vehicle_b_inside = result.vehicle_b_inside || inside_b;
        const auto zone_holder = selectHolder(
            inside_a, inside_b, vehicle_a.id, vehicle_b.id, context);
        result.member_zone_holders.push_back(
            {zone.future_zone_id, zone_holder.first});
        const bool member_cleared =
            passed(vehicle_a, value.generation_a, value.exit_a) ||
            passed(vehicle_b, value.generation_b, value.exit_b);
        all_cleared = all_cleared && member_cleared;
    }

    result.first_enter_s_a = std::isfinite(first_a) ? first_a : 0.0;
    result.first_enter_s_b = std::isfinite(first_b) ? first_b : 0.0;
    result.last_exit_s_a = last_a;
    result.last_exit_s_b = last_b;
    result.all_members_cleared = all_cleared;

    if (result.all_members_cleared) {
        result.holder_id = -1;
        result.waiter_id = -1;
        result.stop_boundary_s = std::min(result.first_enter_s_a,
                                          result.first_enter_s_b);
        result.decision_reason = "cluster_released_all_members";
        return result;
    }

    const auto holder = selectHolder(
        result.vehicle_a_inside, result.vehicle_b_inside,
        vehicle_a.id, vehicle_b.id, context);
    result.holder_id = holder.first;
    result.decision_reason = holder.second;
    if (result.holder_id == vehicle_a.id) {
        result.waiter_id = vehicle_b.id;
        result.stop_boundary_s = result.first_enter_s_b;
    } else if (result.holder_id == vehicle_b.id) {
        result.waiter_id = vehicle_a.id;
        result.stop_boundary_s = result.first_enter_s_a;
    } else {
        result.waiter_id = -1;
        result.stop_boundary_s = std::min(result.first_enter_s_a,
                                          result.first_enter_s_b);
    }

    std::set<int> zone_holders;
    for (const auto& item : result.member_zone_holders) {
        if (item.second >= 0) zone_holders.insert(item.second);
    }
    result.zone_level_mixed_holders = zone_holders.size() > 1;
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
