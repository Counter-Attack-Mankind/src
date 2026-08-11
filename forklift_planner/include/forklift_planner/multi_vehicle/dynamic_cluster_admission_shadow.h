#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "forklift_planner/multi_vehicle/future_cluster_admission_shadow.h"
#include "forklift_planner/multi_vehicle/future_mission_trajectory.h"

namespace forklift_planner {
namespace multi_vehicle {

struct ClusterAdmissionHolderLifecycle {
    int segment_id = -1;
    MissionPhase phase = MissionPhase::DIRECT_TO_B;
    FutureCertainty certainty = FutureCertainty::UNKNOWN;
    int path_generation = -1;
    double cluster_enter_s = 0.0;
    double cluster_exit_s = 0.0;
};

// Shadow representation of the interface a future formal RuleEngine hook
// would consume. It is a resource constraint, not an action and not a real
// reservation.
struct ClusterAdmissionConstraint {
    int cluster_id = -1;
    std::uint64_t horizon_snapshot_id = 0;
    int vehicle_a = -1;
    int vehicle_b = -1;
    int holder_id = -1;
    int waiter_id = -1;
    std::vector<int> member_zone_ids;

    int waiter_path_generation = -1;
    double cluster_enter_s = 0.0;
    double cluster_exit_s = 0.0;
    double earliest_stop_s = 0.0;
    double required_clearance_s = 0.0;

    double evaluated_path_s = 0.0;
    double evaluated_speed = 0.0;
    double curvature_speed_limit = 0.0;
    double approach_speed_upper_bound = 0.0;
    double required_braking_distance = 0.0;
    double available_braking_distance = 0.0;
    double max_accel = 0.0;
    double max_decel = 0.0;
    double dt = 0.0;

    bool admission_feasible = false;
    bool waiter_already_inside = false;
    std::string admission_reason;
    std::vector<ClusterAdmissionHolderLifecycle> holder_lifecycle;
};

struct ClusterAdmissionDecisionShadow {
    int cluster_id = -1;
    int holder_id = -1;
    int waiter_id = -1;
    VehicleAction baseline_action = VehicleAction::NOMINAL;
    VehicleAction constrained_action = VehicleAction::NOMINAL;
    std::string baseline_reason;
    double stop_s = 0.0;
    double distance_to_stop_s = 0.0;
    double required_braking_distance = 0.0;
    bool constraint_active = false;
    bool should_stop_now = false;
    bool action_changed = false;
    bool entered_cluster = false;
    bool stop_boundary_passed = false;
    std::string decision_reason;
};

class ClusterAdmissionEvaluator {
public:
    ClusterAdmissionConstraint buildConstraint(
        const FutureConflictCluster& cluster,
        const ClusterAdmissionShadow& admission,
        const ClusterReservationShadow& arbitration,
        const std::vector<FutureMissionTrajectory>& trajectories,
        const VehicleAgent& vehicle_a,
        const VehicleAgent& vehicle_b,
        double required_clearance_s,
        double dt,
        double max_accel,
        double max_decel,
        double waiter_curvature_speed_limit) const;

    ClusterAdmissionDecisionShadow evaluateDecision(
        const ClusterAdmissionConstraint& constraint,
        const VehicleAgent& waiter,
        double dt,
        double max_decel) const;

    static double stoppingDistance(double speed, double dt,
                                   double max_decel);
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
