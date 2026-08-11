#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "forklift_planner/multi_vehicle/dynamic_cluster_admission_shadow.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class CounterfactualShadowAction { NONE, STOP, GO };

struct ShadowVehicleState {
    int vehicle_id = -1;
    int cluster_id = -1;
    int holder_id = -1;
    int waiter_id = -1;
    CounterfactualShadowAction shadow_action =
        CounterfactualShadowAction::NONE;
    bool cluster_waiting = false;
    bool cluster_released = false;
    double resume_time = -1.0;
    double waiter_stop_s = 0.0;
    double cluster_enter_s = 0.0;
    double waiting_duration = 0.0;
    VehicleAction baseline_action = VehicleAction::NOMINAL;
    VehicleAction constrained_action = VehicleAction::NOMINAL;
    bool action_changed = false;
    double distance_to_stop_s = 0.0;
    double required_braking_distance = 0.0;
    std::string reason;
};

struct CounterfactualClusterStatus {
    int cluster_id = -1;
    std::uint64_t horizon_snapshot_id = 0;
    int vehicle_a = -1;
    int vehicle_b = -1;
    int holder_id = -1;
    int waiter_id = -1;
    std::vector<int> member_zone_ids;
    int holder_path_generation = -1;
    int waiter_path_generation = -1;
    double holder_last_exit_s = 0.0;
    double waiter_enter_s = 0.0;
    double waiter_last_exit_s = 0.0;
    double waiter_stop_s = 0.0;
    double created_time = -1.0;
    double holder_clear_time = -1.0;
    double cluster_release_time = -1.0;
    double waiter_resume_time = -1.0;
    bool waiter_entered_member_zone = false;
    bool admission_braking_feasible = true;
    bool active = false;
    std::vector<ClusterAdmissionHolderLifecycle> holder_lifecycle;
};

struct CounterfactualClusterEvent {
    std::string event;
    CounterfactualClusterStatus status;
};

// Closed-loop counterfactual model used only by an explicitly enabled
// simulation experiment. It owns no RuleEngine state and creates no real
// reservation. The caller may consume its STOP/GO suggestions in a separate
// counterfactual run; normal simulation and real vehicle paths never call it.
class ClusterAdmissionCounterfactualSimulator {
public:
    void refresh(
        const std::vector<FutureConflictCluster>& clusters,
        const std::vector<ClusterAdmissionConstraint>& constraints,
        const std::vector<ClusterReservationShadow>& arbitrations,
        const std::vector<FutureMissionTrajectory>& trajectories,
        const std::vector<VehicleAgent>& vehicles,
        double sim_time);

    std::vector<ShadowVehicleState> step(
        const std::vector<VehicleAgent>& vehicles,
        double sim_time, double dt, double max_decel);

    const std::vector<CounterfactualClusterStatus>& statuses() const {
        return status_cache_;
    }
    std::vector<CounterfactualClusterEvent> takeEvents();

private:
    using Key = std::tuple<int, int, int, int,
                           long long, long long,
                           long long, long long>;

    struct Control {
        CounterfactualClusterStatus status;
        ClusterAdmissionConstraint constraint;
        double waiting_started_time = -1.0;
        bool waiter_stop_reported = false;
    };

    static Key keyFor(const FutureConflictCluster& cluster,
                      const ClusterAdmissionConstraint& constraint,
                      const ClusterReservationShadow& arbitration);
    static const VehicleAgent* vehicleById(
        const std::vector<VehicleAgent>& vehicles, int id);
    static bool trajectoryContainsGeneration(
        const std::vector<FutureMissionTrajectory>& trajectories,
        int vehicle_id, int path_generation);
    void rebuildStatusCache();

    std::map<Key, Control> controls_;
    std::vector<CounterfactualClusterStatus> completed_statuses_;
    std::vector<CounterfactualClusterStatus> status_cache_;
    std::vector<CounterfactualClusterEvent> events_;
    ClusterAdmissionEvaluator evaluator_;
};

const char* counterfactualShadowActionName(CounterfactualShadowAction action);

}  // namespace multi_vehicle
}  // namespace forklift_planner
