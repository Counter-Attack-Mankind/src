#pragma once

#include <string>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/future_conflict_cluster_shadow.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

struct ClusterArbitrationShadowContext {
    int departure_cluster_owner_id = -1;
    int future_a1_owner_id = -1;
    int priority_winner_id = -1;
    bool a_a1_departure = false;
    bool b_a1_departure = false;
    bool a_terminal_docking = false;
    bool b_terminal_docking = false;
};

struct ClusterReservationShadow {
    int cluster_id = -1;
    std::uint64_t horizon_snapshot_id = 0;
    int vehicle_a = -1;
    int vehicle_b = -1;
    int holder_id = -1;
    int waiter_id = -1;
    std::vector<int> member_zone_ids;

    double first_enter_s_a = 0.0;
    double first_enter_s_b = 0.0;
    double last_exit_s_a = 0.0;
    double last_exit_s_b = 0.0;
    double stop_boundary_s = 0.0;

    bool vehicle_a_inside = false;
    bool vehicle_b_inside = false;
    bool all_members_cleared = false;
    std::vector<std::pair<int, int>> member_zone_holders;
    bool zone_level_mixed_holders = false;
    std::string decision_reason;
};

class FutureClusterArbitrationShadow {
public:
    ClusterReservationShadow evaluate(
        const FutureConflictCluster& cluster,
        const VehicleAgent& vehicle_a,
        const VehicleAgent& vehicle_b,
        const ClusterArbitrationShadowContext& context) const;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
