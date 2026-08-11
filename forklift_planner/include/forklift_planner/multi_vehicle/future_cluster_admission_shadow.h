#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "forklift_planner/multi_vehicle/future_cluster_arbitration_shadow.h"

namespace forklift_planner {
namespace multi_vehicle {

// Diagnostic-only cluster admission result.  It never writes VehicleAction,
// RuleEngine state or a real reservation.
struct ClusterAdmissionShadow {
    int cluster_id = -1;
    std::uint64_t horizon_snapshot_id = 0;
    int vehicle_a = -1;
    int vehicle_b = -1;
    int holder_id = -1;
    int waiter_id = -1;
    std::vector<int> member_zone_ids;

    double cluster_enter_s_a = 0.0;
    double cluster_enter_s_b = 0.0;
    double waiter_stop_s = 0.0;
    int entry_path_generation_a = -1;
    int entry_path_generation_b = -1;

    bool prevent_zone_mixing = false;
    bool admission_valid = false;
    bool waiter_before_entry = false;
    bool waiter_already_inside = false;
    bool zone_mixing_observed = false;
    bool entry_generation_consistent = true;
    bool shadow_lock_active = false;
    bool holder_change_suppressed = false;
    double shadow_lock_created_time = -1.0;
    std::string decision_reason;
};

class FutureClusterAdmissionShadow {
public:
    ClusterAdmissionShadow evaluate(
        const FutureConflictCluster& cluster,
        const ClusterReservationShadow& arbitration,
        const VehicleAgent& vehicle_a,
        const VehicleAgent& vehicle_b,
        double stop_buffer) const;
};

// Horizon-to-horizon diagnostic lifecycle.  This is intentionally private to
// the shadow model and cannot create or mutate a RuleEngine reservation.
class FutureClusterAdmissionShadowTracker {
public:
    ClusterAdmissionShadow update(
        ClusterAdmissionShadow current,
        const ClusterReservationShadow& arbitration,
        const VehicleAgent& vehicle_a,
        const VehicleAgent& vehicle_b,
        double sim_time);

private:
    using Key = std::tuple<int, int, int, int,
                           long long, long long,
                           long long, long long>;
    struct Lock {
        int holder_id = -1;
        int waiter_id = -1;
        double waiter_stop_s = 0.0;
        double created_time = -1.0;
    };
    std::map<Key, Lock> locks_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
