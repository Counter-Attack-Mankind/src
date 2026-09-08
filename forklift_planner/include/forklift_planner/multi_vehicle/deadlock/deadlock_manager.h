#pragma once

#include <functional>
#include <string>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class RecoveryPhase {
    NONE,
    RETREAT,
    PASS,
    CLEAR,
    UNRESOLVED,
    ABORT,
};

enum class RecoveryMotion {
    NORMAL,
    HOLD,
    RETREAT,
};

enum class RecoveryKind {
    NORMAL_DEADLOCK,
    A1_LATE_OWNER,
};

struct A1LateOwnerRecoveryRequest {
    int owner_id = -1;
    int transaction_owner_path_gen = -1;
    int owner_departure_path_gen = -1;
    int intruder_id = -1;
    int intruder_path_gen = -1;
    double waiter_stop_s = 0.0;
    PathTrack frozen_owner_track;
    PathTrack frozen_intruder_track;
    std::vector<PotentialConflictZone> closure_zones;

    bool valid() const {
        return owner_id >= 0 && transaction_owner_path_gen >= 0 &&
               owner_departure_path_gen >= 0 && intruder_id >= 0 &&
               intruder_path_gen >= 0 && waiter_stop_s >= 0.0 &&
               !frozen_owner_track.empty() &&
               !frozen_intruder_track.empty() && !closure_zones.empty();
    }
};

struct RecoveryDirective {
    RecoveryKind kind = RecoveryKind::NORMAL_DEADLOCK;
    RecoveryPhase phase = RecoveryPhase::NONE;
    int retreat_vehicle_id = -1;
    int pass_vehicle_id = -1;
    int retreat_path_gen = -1;
    int pass_path_gen = -1;
    double retreat_target_s = 0.0;
    double pass_clear_s = 0.0;
    double retreat_distance = 0.0;
    double estimated_retreat_time = 0.0;
    int cooldown_vehicle_id = -1;
    int cooldown_path_gen = -1;
    double cooldown_remaining = 0.0;
    bool hold_pass_vehicle_during_retreat = true;
    std::string reason;

    bool active() const {
        return phase != RecoveryPhase::NONE && phase != RecoveryPhase::CLEAR;
    }
    bool cooldownActive() const {
        return cooldown_vehicle_id >= 0 && cooldown_remaining > 1e-9;
    }
    RecoveryMotion motionFor(int vehicle_id) const;
};

struct DeadlockPairGeometry {
    int vehicle_a = -1;
    int vehicle_b = -1;
    int path_gen_a = -1;
    int path_gen_b = -1;
    int preferred_priority_vehicle_id = -1;
    std::vector<PotentialConflictZone> zones;
};

class DeadlockManager {
public:
    struct CandidateState {
        bool valid = false;
        int vehicle_a = -1;
        int vehicle_b = -1;
        int path_gen_a = -1;
        int path_gen_b = -1;
        double duration = 0.0;
        double anchor_s_a = 0.0;
        double anchor_s_b = 0.0;
    };

    struct TransactionState {
        RecoveryKind kind = RecoveryKind::NORMAL_DEADLOCK;
        RecoveryPhase phase = RecoveryPhase::NONE;
        int retreat_vehicle_id = -1;
        int pass_vehicle_id = -1;
        int retreat_path_gen = -1;
        int pass_path_gen = -1;
        double retreat_target_s = 0.0;
        double pass_clear_s = 0.0;
        double retreat_distance = 0.0;
        double estimated_retreat_time = 0.0;
        double pass_confirmation_elapsed = 0.0;
        double pass_track_length = 0.0;
        bool hold_pass_vehicle_during_retreat = true;
        int a1_transaction_owner_path_gen = -1;
        int a1_owner_departure_path_gen = -1;
        double a1_waiter_stop_s = 0.0;
        PathTrack a1_frozen_owner_track;
        PathTrack a1_frozen_intruder_track;
        std::vector<PotentialConflictZone> a1_closure_zones;
        std::string reason;
    };

    struct CooldownState {
        int vehicle_id = -1;
        int path_gen = -1;
        double remaining = 0.0;
    };

    struct Snapshot {
        CandidateState candidate;
        TransactionState transaction;
        CooldownState cooldown;
        RecoveryDirective directive;
    };

    DeadlockManager(const MapParam& map_param,
                    const MultiVehicleConfig& config);

    void setLogSink(const std::function<void(const std::string&)>& sink) {
        log_sink_ = sink;
    }

    void update(const std::vector<VehicleAgent>& vehicles,
                const std::vector<DeadlockPairGeometry>& pair_geometry,
                double dt, bool emit_logs);

    void requestA1LateOwnerRecovery(
        const A1LateOwnerRecoveryRequest& request,
        const std::vector<VehicleAgent>& vehicles, bool emit_logs);

    const RecoveryDirective& directive() const { return directive_; }

    Snapshot snapshot() const;
    void restore(const Snapshot& snapshot);

private:
    struct RetreatEvaluation {
        bool feasible = false;
        int retreat_vehicle_id = -1;
        int pass_vehicle_id = -1;
        double target_s = 0.0;
        double pass_clear_s = 0.0;
        double distance = 0.0;
        std::string reason;
    };

    const VehicleAgent* vehicleById(const std::vector<VehicleAgent>& vehicles,
                                    int id) const;
    const DeadlockPairGeometry* geometryFor(
        const std::vector<DeadlockPairGeometry>& geometry,
        int vehicle_a, int vehicle_b) const;
    RetreatEvaluation evaluateRetreat(
        const VehicleAgent& retreat, const VehicleAgent& passer,
        const std::vector<VehicleAgent>& vehicles,
        const DeadlockPairGeometry& geometry) const;
    bool retreatSweepClear(const VehicleAgent& retreat,
                           const VehicleAgent& passer,
                           const std::vector<VehicleAgent>& vehicles,
                           double target_s) const;
    bool retreatPoseClearsPassCorridor(const VehicleAgent& retreat,
                                       const VehicleAgent& passer,
                                       double retreat_s,
                                       double pass_clear_s) const;
    bool lateOwnerRetreatSweepClear(
        const VehicleAgent& intruder, const VehicleAgent& owner,
        const std::vector<VehicleAgent>& vehicles, double target_s,
        bool ignore_owner) const;
    bool lateOwnerPoseClearsClosure(
        const PathTrack& intruder_track, double intruder_s,
        const PathTrack& owner_track,
        const std::vector<PotentialConflictZone>& closure_zones) const;
    bool lateOwnerRetreatConflictsWithOwnerPath(
        const VehicleAgent& intruder, double target_s,
        const VehicleAgent& owner) const;
    void refreshDirective();
    void emit(const char* event, const std::string& details, bool enabled) const;
    void abort(const std::string& reason, bool emit_logs);
    void clearSuccessfulRecovery(const VehicleAgent* retreat,
                                 const VehicleAgent* passer,
                                 const std::string& reason,
                                 bool emit_logs);

    const MapParam& map_param_;
    const MultiVehicleConfig& config_;
    CandidateState candidate_;
    TransactionState transaction_;
    CooldownState cooldown_;
    RecoveryDirective directive_;
    std::function<void(const std::string&)> log_sink_;
};

const char* recoveryPhaseName(RecoveryPhase phase);

}  // namespace multi_vehicle
}  // namespace forklift_planner
