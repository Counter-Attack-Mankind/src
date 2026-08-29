#pragma once

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/future_a1_policy.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

struct FutureA1Commitment {
    int owner_id = -1;
    int owner_path_gen = -1;
    double predicted_a1_arrival_time = -1.0;
    double predicted_to_b_time = -1.0;

    bool valid() const {
        return owner_id >= 0 && owner_path_gen >= 0 &&
               predicted_a1_arrival_time >= 0.0 &&
               predicted_to_b_time >= predicted_a1_arrival_time;
    }
};

struct FutureA1ArrivalCandidate {
    int vehicle_id = -1;
    int path_gen = -1;
    double arrival_time = -1.0;
    double to_b_time = -1.0;
};

struct FutureA1Update {
    FutureA1Commitment previous;
    FutureA1Commitment current;
    std::string reason;
};

struct DepartureClusterCommitment {
    int owner_id = -1;
    int owner_path_gen = -1;
    int other_id = -1;
    int other_path_gen = -1;
    std::vector<size_t> seed_indices;
    std::vector<size_t> cluster_indices;
    std::vector<FutureA1ConflictInterval> intervals;
    double waiter_physical_entry_s = 0.0;
    double waiter_control_stop_s = 0.0;
    double owner_release_exit_s = 0.0;
    double other_release_exit_s = 0.0;
    bool active = false;
    bool handed_off_from_future = false;
    bool handoff_already_inside = false;
    bool hold_logged = false;
};

enum class A1AuthoritySource {
    NONE,
    FUTURE_COMMITMENT,
    DEPARTURE_CLUSTER,
    ACTUAL_OCCUPANCY,
};

struct A1PairAuthority {
    bool protected_pair = false;
    int owner_id = -1;
    A1AuthoritySource source = A1AuthoritySource::NONE;
};

struct A1ActionRequest {
    int vehicle_id = -1;
    VehicleAction action = VehicleAction::NOMINAL;
    int blocker_id = -1;
    std::string reason;
};

struct A1LaunchAdmission {
    bool departure_resource_conflict = false;
    bool actual_occupancy_priority = false;
    bool owner_uses_pending_preview = false;
    size_t protected_zone_count = 0;
};

struct A1CoordinationSnapshot {
    std::map<std::pair<int, int>, DepartureClusterCommitment>
        departure_clusters;
};

class A1Coordinator {
public:
    using ConflictZone = PotentialConflictZone;
    using ComputeZones = std::function<std::vector<ConflictZone>(
        const VehicleAgent&, const VehicleAgent&)>;
    using PriorityWinner = std::function<int(int, int)>;

    A1Coordinator(const MapParam& map_param,
                  const MultiVehicleConfig& config);

    void setCoordLogSink(const std::function<void(const std::string&)>& sink) {
        coord_log_sink_ = sink;
    }
    void setDebugLogPrefix(const std::string& prefix) {
        debug_log_prefix_ = prefix;
    }
    void resetPlanDiagnostics();

    FutureA1Update updateFutureOwner(
        const std::vector<FutureA1ArrivalCandidate>& candidates,
        const std::vector<VehicleAgent>& vehicles, double tie_window,
        const PriorityWinner& priority_winner);
    void clearFutureCommitment();
    void setFutureCommitment(const FutureA1Commitment& commitment);
    const FutureA1Commitment& futureCommitment() const {
        return future_commitment_;
    }

    A1PairAuthority futureAuthorityForPair(
        const VehicleAgent& a, const VehicleAgent& b,
        const ComputeZones& compute_full,
        const ComputeZones& canonical_zones);
    A1PairAuthority departureAuthorityForPair(
        const VehicleAgent& a, const VehicleAgent& b) const;

    void refreshDepartureClusters(std::vector<VehicleAgent>& vehicles,
                                  const ComputeZones& canonical_zones);
    std::vector<A1ActionRequest> enforceFutureAdmission(
        std::vector<VehicleAgent>& vehicles, double dt,
        const ComputeZones& compute_full,
        const ComputeZones& canonical_zones);
    std::vector<A1ActionRequest> enforceDepartureClusters(
        std::vector<VehicleAgent>& vehicles, double dt);

    A1LaunchAdmission checkLaunchAdmission(
        const VehicleAgent& service_owner,
        const VehicleAgent& launch_candidate,
        const ComputeZones& compute_full) const;

    size_t activeDepartureClusterCount(int owner_id) const;
    const std::map<std::pair<int, int>, DepartureClusterCommitment>&
    departureClusters() const {
        return departure_clusters_;
    }

    bool shouldLogDecision(const VehicleAgent& vehicle, int blocker_id);
    void logDecision(const VehicleAgent& vehicle,
                     const VehicleAgent* blocker, int blocker_id) const;

    A1CoordinationSnapshot snapshot() const;
    void restore(const A1CoordinationSnapshot& snapshot);

private:
    struct FutureA1ZoneSelection {
        std::vector<ConflictZone> normalized_zones;
        std::vector<size_t> seed_indices;
        std::vector<size_t> protected_indices;
        int upstream_index = -1;
        bool other_already_inside = false;
    };

    struct ConflictCacheEntry {
        int gen_lo = -1;
        int gen_hi = -1;
        std::vector<ConflictZone> blocks;
    };

    struct PhysicalProgress {
        bool valid = false;
        bool heading_aligned = false;
        double path_s = 0.0;
    };

    FutureA1Commitment selectOwner(
        const std::vector<FutureA1ArrivalCandidate>& candidates,
        double tie_window, const PriorityWinner& priority_winner) const;
    FutureA1Commitment retainOwner(
        const std::vector<FutureA1ArrivalCandidate>& candidates,
        const std::vector<VehicleAgent>& vehicles,
        std::string& reason) const;
    FutureA1ZoneSelection selectProtectedZones(
        const std::vector<ConflictZone>& canonical_zones,
        bool preview_is_lo, double protected_until,
        double other_path_s) const;
    PhysicalProgress physicalProgress(const VehicleAgent& vehicle) const;
    double occupancyPathS(const VehicleAgent& vehicle,
                          bool* confirmed = nullptr) const;
    void updatePhysicalDepartureState(VehicleAgent& owner) const;

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
    FutureA1Commitment future_commitment_;
    std::map<std::pair<int, int>, DepartureClusterCommitment>
        departure_clusters_;
    mutable std::map<std::pair<int, int>, ConflictCacheEntry>
        future_conflict_cache_;
    std::set<std::pair<int, int>> admission_logged_;
    std::set<std::tuple<int, int, int, int>> decision_logs_;
    int admission_log_owner_id_ = -1;
    int admission_log_owner_path_gen_ = -1;
    std::function<void(const std::string&)> coord_log_sink_;
    std::string debug_log_prefix_;
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
