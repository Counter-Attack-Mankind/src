#include "forklift_planner/multi_vehicle/future_cluster_admission_shadow.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

struct Entry {
    double s = std::numeric_limits<double>::infinity();
    int generation = -1;
    bool generation_consistent = true;
};

void includeEntry(Entry& entry, double s, int generation) {
    if (s < entry.s - 1e-9) {
        entry.s = s;
        entry.generation = generation;
        entry.generation_consistent = true;
    } else if (std::abs(s - entry.s) <= 1e-9 &&
               entry.generation != generation) {
        entry.generation_consistent = false;
    }
}

bool beforeEntry(const VehicleAgent& vehicle, const Entry& entry) {
    if (!std::isfinite(entry.s) || entry.generation < 0 ||
        !entry.generation_consistent) {
        return false;
    }
    if (vehicle.path_gen < entry.generation) return true;
    return vehicle.path_gen == entry.generation &&
           vehicle.path_s <= entry.s + 1e-9;
}

long long quantize(double value) {
    return static_cast<long long>(std::llround(value * 1000.0));
}

}  // namespace

ClusterAdmissionShadow FutureClusterAdmissionShadow::evaluate(
    const FutureConflictCluster& cluster,
    const ClusterReservationShadow& arbitration,
    const VehicleAgent& vehicle_a,
    const VehicleAgent& vehicle_b,
    double stop_buffer) const {
    ClusterAdmissionShadow result;
    result.cluster_id = cluster.cluster_id;
    result.horizon_snapshot_id = cluster.horizon_snapshot_id;
    result.vehicle_a = vehicle_a.id;
    result.vehicle_b = vehicle_b.id;
    result.holder_id = arbitration.holder_id;
    result.waiter_id = arbitration.waiter_id;
    result.member_zone_ids = cluster.member_zone_ids;

    Entry entry_a;
    Entry entry_b;
    for (const FutureConflictZone& zone : cluster.member_zones) {
        if (zone.vehicle_a == vehicle_a.id) {
            includeEntry(entry_a, zone.s_a_enter,
                         zone.path_generation_a);
            includeEntry(entry_b, zone.s_b_enter,
                         zone.path_generation_b);
        } else {
            includeEntry(entry_a, zone.s_b_enter,
                         zone.path_generation_b);
            includeEntry(entry_b, zone.s_a_enter,
                         zone.path_generation_a);
        }
    }
    result.cluster_enter_s_a =
        std::isfinite(entry_a.s) ? entry_a.s : 0.0;
    result.cluster_enter_s_b =
        std::isfinite(entry_b.s) ? entry_b.s : 0.0;
    result.entry_path_generation_a = entry_a.generation;
    result.entry_path_generation_b = entry_b.generation;
    result.entry_generation_consistent =
        entry_a.generation_consistent && entry_b.generation_consistent;

    const bool a_before = beforeEntry(vehicle_a, entry_a);
    const bool b_before = beforeEntry(vehicle_b, entry_b);
    double waiter_enter = std::min(result.cluster_enter_s_a,
                                   result.cluster_enter_s_b);
    if (result.waiter_id == vehicle_a.id) {
        waiter_enter = result.cluster_enter_s_a;
        result.waiter_before_entry = a_before;
        result.waiter_already_inside = arbitration.vehicle_a_inside;
    } else if (result.waiter_id == vehicle_b.id) {
        waiter_enter = result.cluster_enter_s_b;
        result.waiter_before_entry = b_before;
        result.waiter_already_inside = arbitration.vehicle_b_inside;
    }
    result.waiter_stop_s = std::max(
        0.0, waiter_enter - std::max(0.0, stop_buffer));
    result.zone_mixing_observed =
        arbitration.zone_level_mixed_holders &&
        arbitration.vehicle_a_inside && arbitration.vehicle_b_inside;

    const bool valid_pair =
        result.holder_id >= 0 && result.waiter_id >= 0 &&
        result.holder_id != result.waiter_id;
    result.admission_valid =
        valid_pair && !arbitration.all_members_cleared &&
        result.entry_generation_consistent &&
        result.waiter_before_entry && !result.waiter_already_inside &&
        result.waiter_stop_s < waiter_enter - 1e-9;

    // prevent_zone_mixing is explicitly counterfactual.  A mixed actual
    // snapshot is too late to enforce, but proves that the earlier common
    // boundary would have prevented the waiter entering the second member.
    result.prevent_zone_mixing =
        valid_pair && !arbitration.all_members_cleared &&
        result.entry_generation_consistent &&
        result.waiter_stop_s < waiter_enter - 1e-9 &&
        (result.admission_valid || result.zone_mixing_observed);

    if (arbitration.all_members_cleared) {
        result.decision_reason = "cluster_released_all_members";
    } else if (!valid_pair) {
        result.decision_reason = "invalid_cluster_arbitration";
    } else if (!result.entry_generation_consistent) {
        result.decision_reason = "ambiguous_entry_path_generation";
    } else if (result.zone_mixing_observed) {
        result.decision_reason =
            "counterfactual_pre_entry_admission_would_prevent_mixing";
    } else if (result.waiter_already_inside) {
        result.decision_reason = "admission_too_late_waiter_inside";
    } else if (result.admission_valid) {
        result.decision_reason = "pre_entry_cluster_admission";
    } else {
        result.decision_reason = "admission_not_available";
    }
    return result;
}

ClusterAdmissionShadow FutureClusterAdmissionShadowTracker::update(
    ClusterAdmissionShadow current,
    const ClusterReservationShadow& arbitration,
    const VehicleAgent& vehicle_a,
    const VehicleAgent& vehicle_b,
    double sim_time) {
    const Key key{
        std::min(current.vehicle_a, current.vehicle_b),
        std::max(current.vehicle_a, current.vehicle_b),
        current.entry_path_generation_a,
        current.entry_path_generation_b,
        quantize(current.cluster_enter_s_a),
        quantize(current.cluster_enter_s_b),
        quantize(arbitration.last_exit_s_a),
        quantize(arbitration.last_exit_s_b)};

    if (arbitration.all_members_cleared) {
        locks_.erase(key);
        return current;
    }

    auto lock = locks_.find(key);
    if (lock == locks_.end() && current.admission_valid) {
        lock = locks_.emplace(
            key, Lock{current.holder_id, current.waiter_id,
                      current.waiter_stop_s, sim_time}).first;
        current.decision_reason = "shadow_admission_create";
    }
    if (lock == locks_.end()) return current;

    const int refreshed_holder = current.holder_id;
    current.holder_id = lock->second.holder_id;
    current.waiter_id = lock->second.waiter_id;
    current.waiter_stop_s = lock->second.waiter_stop_s;
    current.shadow_lock_active = true;
    current.shadow_lock_created_time = lock->second.created_time;
    current.holder_change_suppressed =
        refreshed_holder >= 0 && refreshed_holder != current.holder_id;

    const VehicleAgent& waiter = current.waiter_id == vehicle_a.id
        ? vehicle_a : vehicle_b;
    const bool waiter_is_a = current.waiter_id == vehicle_a.id;
    const double waiter_enter = waiter_is_a
        ? current.cluster_enter_s_a : current.cluster_enter_s_b;
    const int waiter_generation = waiter_is_a
        ? current.entry_path_generation_a
        : current.entry_path_generation_b;
    current.waiter_before_entry =
        waiter.path_gen < waiter_generation ||
        (waiter.path_gen == waiter_generation &&
         waiter.path_s <= waiter_enter + 1e-9);
    current.waiter_already_inside = waiter_is_a
        ? arbitration.vehicle_a_inside : arbitration.vehicle_b_inside;
    current.admission_valid =
        current.entry_generation_consistent &&
        current.waiter_before_entry && !current.waiter_already_inside &&
        current.waiter_stop_s < waiter_enter - 1e-9;
    current.prevent_zone_mixing =
        current.entry_generation_consistent &&
        current.waiter_stop_s < waiter_enter - 1e-9;

    if (current.zone_mixing_observed) {
        current.decision_reason =
            "shadow_lock_counterfactual_prevented_zone_mixing";
    } else if (current.waiter_already_inside) {
        current.decision_reason = "shadow_admission_violation_observed";
    } else if (current.holder_change_suppressed) {
        current.decision_reason = "shadow_admission_hold_suppressed_change";
    } else if (current.decision_reason != "shadow_admission_create") {
        current.decision_reason = "shadow_admission_hold";
    }
    return current;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
