#include "forklift_planner/multi_vehicle/rule_engine.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace forklift_planner::multi_vehicle;

namespace {

int fail(const std::string& message) {
    std::cerr << "dynamic_speed_rule_engine_test: " << message << '\n';
    return 1;
}

RoughWp wp(double x, double y, double theta) {
    return RoughWp{x, y, theta, WpType::FORWARD};
}

VehicleAgent crossingVehicle(int id, double approach, bool vertical,
                             double speed = 0.0) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.mission_phase = MissionPhase::TO_A1;
    result.path_gen = 1;
    result.current_speed = speed;
    result.track.set(vertical
        ? RoughPath{wp(0.0, -approach, 1.5707963267948966),
                    wp(0.0, 2.0, 1.5707963267948966)}
        : RoughPath{wp(-approach, 0.0, 0.0),
                    wp(2.0, 0.0, 0.0)});
    return result;
}

VehicleAgent diagonalVehicle(int id, double approach, double speed = 0.0) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = 1;
    result.current_speed = speed;
    constexpr double kQuarterPi = 0.7853981633974483;
    result.track.set(RoughPath{
        wp(-approach, -approach, kQuarterPi),
        wp(2.0, 2.0, kQuarterPi)});
    return result;
}

VehicleAgent laneVehicle(int id, double path_s, double speed) {
    VehicleAgent result;
    result.id = id;
    result.mode = VehicleMode::ACTIVE;
    result.action = VehicleAction::NOMINAL;
    result.requested_action = VehicleAction::NOMINAL;
    result.path_gen = 1;
    result.path_s = path_s;
    result.current_speed = speed;
    result.track.set(RoughPath{wp(0.0, 0.0, 0.0),
                               wp(4.0, 0.0, 0.0)});
    return result;
}

bool hasDynamicReason(const std::vector<VehicleAgent>& vehicles,
                      VehicleAction action) {
    for (const VehicleAgent& vehicle : vehicles) {
        if (vehicle.requested_action == action &&
            vehicle.reason.rfind("dynamic_speed_", 0) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    MapParam map_param;
    MultiVehicleConfig config;
    config.prediction_horizon = 15.0;
    config.prediction_step = 0.05;

    // Slot departure admission gives an already ACTIVE road vehicle priority
    // only when synchronized OBB overlap occurs before the candidate clears
    // its source-slot prefix. A later conflict remains rolling-coordinator
    // work and must not hold the parked vehicle.
    RuleEngine admission_engine(map_param, config);
    VehicleAgent launch_candidate = crossingVehicle(20, 0.0, false);
    launch_candidate.track.set(RoughPath{
        wp(0.0, 0.0, 0.0), wp(2.0, 0.0, 0.0)});
    launch_candidate.slot_departure_clear_s = 0.5;
    VehicleAgent immediate_occupant = crossingVehicle(21, 0.0, true);
    immediate_occupant.track.set(RoughPath{
        wp(0.0, 0.0, 1.5707963267948966),
        wp(0.0, 2.0, 1.5707963267948966)});
    const auto immediate_admission =
        admission_engine.checkSlotDepartureAdmission(
            nullptr, launch_candidate, {immediate_occupant}, 15.0);
    if (immediate_admission.clear ||
        !immediate_admission.ordinary_road_conflict ||
        immediate_admission.blocker_id != immediate_occupant.id ||
        immediate_admission.candidate_conflict_s >
            launch_candidate.slot_departure_clear_s + 1e-9) {
        return fail("immediate slot departure conflict was not held");
    }

    VehicleAgent later_occupant = crossingVehicle(22, 0.0, true);
    later_occupant.track.set(RoughPath{
        wp(1.5, -1.5, 1.5707963267948966),
        wp(1.5, 2.0, 1.5707963267948966)});
    const auto later_admission =
        admission_engine.checkSlotDepartureAdmission(
            nullptr, launch_candidate, {later_occupant}, 15.0);
    if (!later_admission.clear ||
        later_admission.ordinary_road_conflict) {
        return fail("post-slot future conflict over-held departure");
    }

    VehicleAgent service_owner = crossingVehicle(23, 1.0, true);
    service_owner.pending_dropoff_valid = true;
    service_owner.pending_dropoff_track = later_occupant.track;
    service_owner.a1_departure_priority_until_s =
        service_owner.pending_dropoff_track.length();
    const auto far_a1 = admission_engine.checkA1LaunchAdmission(
        service_owner, launch_candidate);
    if (far_a1.departure_resource_conflict) {
        return fail("far A1 departure closure over-held slot launch");
    }
    service_owner.pending_dropoff_track = immediate_occupant.track;
    service_owner.a1_departure_priority_until_s =
        service_owner.pending_dropoff_track.length();
    const auto near_a1 = admission_engine.checkA1LaunchAdmission(
        service_owner, launch_candidate);
    if (!near_a1.departure_resource_conflict) {
        return fail("immediate A1 departure prefix conflict was not held");
    }

    RuleEngine far_engine(map_param, config);
    std::vector<VehicleAgent> far{
        crossingVehicle(0, 2.50, false),
        crossingVehicle(1, 2.90, true)};
    far_engine.decide(far, 0.1, 15.0);
    if (far_engine.dynamicSpeedMetrics().far_decisions == 0 ||
        far[0].requested_action != VehicleAction::NOMINAL ||
        far[1].requested_action != VehicleAction::NOMINAL ||
        !far_engine.snapshot().reservations.empty()) {
        return fail("FAR did not remain reservation-free NOMINAL");
    }

    RuleEngine mid_engine(map_param, config);
    std::vector<VehicleAgent> mid{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    mid_engine.decide(mid, 0.1, 15.0);
    if (mid_engine.dynamicSpeedMetrics().mid_decisions == 0 ||
        !hasDynamicReason(mid, VehicleAction::YIELD) ||
        !mid_engine.snapshot().reservations.empty()) {
        return fail("MID did not accept one reservation-free YIELD");
    }

    RuleEngine near_engine(map_param, config);
    std::vector<VehicleAgent> near{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.79, true)};
    near_engine.decide(near, 0.1, 15.0);
    if (near_engine.dynamicSpeedMetrics().near_decisions == 0 ||
        !hasDynamicReason(near, VehicleAction::CREEP) ||
        !near_engine.snapshot().reservations.empty()) {
        return fail("NEAR did not jump directly to reservation-free CREEP");
    }

    // Braking safety remains a direct STOP motion action, but no longer
    // creates ordinary-road holder/waiter ownership.
    RuleEngine emergency_engine(map_param, config);
    std::vector<VehicleAgent> emergency{
        crossingVehicle(0, 0.30, false, config.nominal_speed),
        crossingVehicle(1, 0.30, true, config.nominal_speed)};
    emergency_engine.decide(emergency, 0.1, 15.0);
    const auto emergency_state = emergency_engine.snapshot();
    if (emergency_engine.dynamicSpeedMetrics().emergency_stop_decisions == 0 ||
        !hasDynamicReason(emergency, VehicleAction::STOP) ||
        !emergency_state.reservations.empty()) {
        return fail("braking emergency did not remain reservation-free STOP");
    }

    RuleEngine reserved_engine(map_param, config);
    std::vector<VehicleAgent> reserved{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    RuleEngine::SimSnapshot reservation_state;
    RuleEngine::ConflictReservation reservation;
    reservation.owner_id = 0;
    reservation.gen_lo = 1;
    reservation.gen_hi = 1;
    reservation.enter_lo = 0.10;
    reservation.exit_lo = 0.70;
    reservation.enter_hi = 0.40;
    reservation.exit_hi = 1.00;
    reservation.create_reason = "already_inside";
    reservation_state.reservations[{0, 1}] = reservation;
    reserved_engine.restore(reservation_state);
    reserved_engine.decide(reserved, 0.1, 15.0);
    if (!reserved_engine.snapshot().reservations.empty() ||
        reserved_engine.dynamicSpeedMetrics().reservation_deletes == 0 ||
        reserved_engine.dynamicSpeedMetrics().existing_reservation_skips != 0 ||
        reserved_engine.dynamicSpeedMetrics().baseline_conflicts == 0) {
        return fail("ordinary already-inside reservation was not retired");
    }

    // A single vehicle departure flag is not pair-level A1 authority.
    RuleEngine departure_flag_engine(map_param, config);
    std::vector<VehicleAgent> departure_flag{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    departure_flag[0].mission_phase = MissionPhase::TO_B;
    departure_flag[1].mission_phase = MissionPhase::TO_B;
    departure_flag[0].a1_departure_committed = true;
    departure_flag[0].a1_departure_priority_until_s = 1.0;
    departure_flag_engine.decide(departure_flag, 0.1, 15.0);
    if (!departure_flag_engine.snapshot().reservations.empty() ||
        departure_flag_engine.dynamicSpeedMetrics().baseline_conflicts == 0 ||
        departure_flag_engine.dynamicSpeedMetrics().a1_fallbacks != 0) {
        return fail("single A1 departure flag still captured an ordinary pair");
    }

    // Future owner identity without a real future-exit conflict remains an
    // ordinary crossing and must use rolling dynamic coordination.
    RuleEngine identity_engine(map_param, config);
    std::vector<VehicleAgent> identity{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    identity[0].pending_dropoff_valid = true;
    identity[0].pending_dropoff_track.set(
        RoughPath{wp(10.0, 10.0, 0.0), wp(12.0, 10.0, 0.0)});
    identity[0].a1_departure_priority_until_s = 1.0;
    RuleEngine::FutureA1Commitment identity_owner;
    identity_owner.owner_id = 0;
    identity_owner.owner_path_gen = 1;
    identity_owner.predicted_a1_arrival_time = 5.0;
    identity_owner.predicted_to_b_time = 10.0;
    identity_engine.setFutureA1Commitment(identity_owner);
    identity_engine.decide(identity, 0.1, 15.0);
    if (!identity_engine.snapshot().reservations.empty() ||
        identity_engine.dynamicSpeedMetrics().mid_decisions == 0 ||
        identity_engine.dynamicSpeedMetrics().a1_fallbacks != 0) {
        return fail("Future A1 owner identity still captured an ordinary pair");
    }

    // An inactive staged handoff is retained for TO_B activation, but it is
    // not current pair authority while the future exit has no real conflict.
    RuleEngine staged_engine(map_param, config);
    std::vector<VehicleAgent> staged = identity;
    RuleEngine::SimSnapshot staged_state;
    RuleEngine::DepartureClusterCommitment staged_commitment;
    staged_commitment.owner_id = 0;
    staged_commitment.owner_path_gen = 2;
    staged_commitment.other_id = 1;
    staged_commitment.other_path_gen = 1;
    staged_commitment.active = false;
    staged_state.departure_clusters[{0, 1}] = staged_commitment;
    staged_engine.restore(staged_state);
    staged_engine.setFutureA1Commitment(identity_owner);
    staged_engine.decide(staged, 0.1, 15.0);
    if (!staged_engine.snapshot().reservations.empty() ||
        staged_engine.dynamicSpeedMetrics().mid_decisions == 0 ||
        staged_engine.dynamicSpeedMetrics().a1_fallbacks != 0) {
        return fail("inactive staged handoff still captured an ordinary pair");
    }

    // A valid active departure cluster remains strong pair-level A1
    // authority and retains the legacy A1 reservation chain.
    RuleEngine active_cluster_engine(map_param, config);
    std::vector<VehicleAgent> active_cluster{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    active_cluster[0].mission_phase = MissionPhase::TO_B;
    RuleEngine::SimSnapshot active_state;
    RuleEngine::DepartureClusterCommitment active_commitment;
    active_commitment.owner_id = 0;
    active_commitment.owner_path_gen = 1;
    active_commitment.other_id = 1;
    active_commitment.other_path_gen = 1;
    active_commitment.intervals.push_back(
        FutureA1ConflictInterval{0.10, 1.50, 0.40, 1.60});
    active_commitment.owner_release_exit_s = 1.50;
    active_commitment.other_release_exit_s = 1.60;
    active_commitment.active = true;
    active_state.departure_clusters[{0, 1}] = active_commitment;
    active_cluster_engine.restore(active_state);
    active_cluster_engine.decide(active_cluster, 0.1, 15.0);
    if (active_cluster_engine.snapshot().reservations.empty() ||
        active_cluster_engine.snapshot().reservations.begin()->second.
                create_reason != "a1_related" ||
        active_cluster_engine.dynamicSpeedMetrics().a1_fallbacks == 0) {
        return fail("active departure cluster lost A1 pair authority");
    }

    // PICKUP_DWELL is inactive for pairwise motion, but future admission must
    // still stage the synthetic TO_B conflict for a TO_A1 vehicle.
    RuleEngine pickup_engine(map_param, config);
    std::vector<VehicleAgent> pickup{
        crossingVehicle(0, 0.30, false),
        crossingVehicle(1, 0.70, true)};
    pickup[0].mode = VehicleMode::DWELL;
    pickup[0].mission_phase = MissionPhase::PICKUP_DWELL;
    pickup[0].pending_dropoff_valid = true;
    pickup[0].pending_dropoff_track = pickup[0].track;
    pickup[0].a1_departure_priority_until_s = 1.0;
    RuleEngine::FutureA1Commitment pickup_owner;
    pickup_owner.owner_id = 0;
    pickup_owner.owner_path_gen = 1;
    pickup_owner.predicted_a1_arrival_time = 0.0;
    pickup_owner.predicted_to_b_time = 5.0;
    pickup_engine.setFutureA1Commitment(pickup_owner);
    pickup_engine.decide(pickup, 0.1, 15.0);
    const auto pickup_state = pickup_engine.snapshot();
    if (pickup_state.departure_clusters.empty() ||
        pickup_state.departure_clusters.begin()->second.active) {
        return fail("PICKUP_DWELL future departure protection was not staged");
    }

    // Three mutually crossing vehicles exercise all three pairwise dynamic
    // calls. Their requests must aggregate without a legacy reservation.
    RuleEngine multi_engine(map_param, config);
    std::vector<VehicleAgent> multi{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.70, true),
        diagonalVehicle(2, 1.10)};
    multi_engine.decide(multi, 0.1, 15.0);
    const auto& multi_metrics = multi_engine.dynamicSpeedMetrics();
    if (!multi_engine.snapshot().reservations.empty() ||
        multi_metrics.baseline_conflicts < 3 ||
        multi_metrics.reservation_create_multi_vehicle != 0 ||
        multi_engine.lastRollingDynamicDecision().targets.empty()) {
        return fail("three-vehicle pairs did not use dynamic aggregation");
    }
    std::vector<int> target_ids;
    for (const auto& target :
         multi_engine.lastRollingDynamicDecision().targets) {
        for (int id : target_ids) {
            if (id == target.vehicle_id) {
                return fail("multi-pair aggregate stored duplicate target");
            }
        }
        target_ids.push_back(target.vehicle_id);
    }

    // Same-direction classification uses current travel direction/lateral
    // alignment and stable longitudinal order. The physical front vehicle is
    // always the dynamic winner even when the rear vehicle has the smaller id.
    RuleEngine following_engine(map_param, config);
    std::vector<VehicleAgent> following{
        laneVehicle(0, 0.20, config.nominal_speed),
        laneVehicle(1, 0.48, 0.0)};
    following_engine.decide(following, 0.1, 15.0);
    if (following_engine.dynamicSpeedMetrics().same_direction_conflicts == 0 ||
        following[1].requested_action != VehicleAction::NOMINAL ||
        following[0].requested_action == VehicleAction::NOMINAL ||
        following[0].blocker_id != following[1].id ||
        following_engine.dynamicSpeedMetrics().
                duplicate_pair_authority_overrides != 0) {
        std::cerr << "same_direction_conflicts="
                  << following_engine.dynamicSpeedMetrics().
                         same_direction_conflicts
                  << " rear_action="
                  << static_cast<int>(following[0].requested_action)
                  << " front_action="
                  << static_cast<int>(following[1].requested_action)
                  << " rear_blocker=" << following[0].blocker_id
                  << " duplicate="
                  << following_engine.dynamicSpeedMetrics().
                         duplicate_pair_authority_overrides
                  << '\n';
        return fail("same-direction front/rear authority was not stable");
    }

    // A clear next rolling period returns to NOMINAL and reports recovery.
    RuleEngine recovery_engine(map_param, config);
    std::vector<VehicleAgent> recovery{
        crossingVehicle(0, 1.50, false),
        crossingVehicle(1, 1.90, true)};
    recovery_engine.decide(recovery, 0.1, 15.0);
    const auto prefix_a = predictTrajectory(
        recovery[0], map_param, config, VehicleAction::NOMINAL, 15.0);
    recovery[0].path_s = prefix_a.back().s;
    recovery[1].track.set(RoughPath{wp(10.0, 10.0, 0.0),
                                    wp(12.0, 10.0, 0.0)});
    recovery[1].path_gen += 1;
    recovery_engine.decide(recovery, 0.1, 15.0);
    if (hasDynamicReason(recovery, VehicleAction::YIELD) ||
        hasDynamicReason(recovery, VehicleAction::CREEP) ||
        !recovery_engine.snapshot().reservations.empty()) {
        return fail("next real rolling decision did not return to NOMINAL");
    }

    std::cout << "dynamic_speed_rule_engine_test: PASS\n";
    return 0;
}
