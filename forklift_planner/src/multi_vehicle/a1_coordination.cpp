#include "forklift_planner/multi_vehicle/a1_coordination.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <ros/ros.h>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

const char* missionPhaseName(MissionPhase phase) {
    switch (phase) {
        case MissionPhase::DIRECT_TO_B: return "DIRECT_TO_B";
        case MissionPhase::TO_A1: return "TO_A1";
        case MissionPhase::PICKUP_DWELL: return "PICKUP_DWELL";
        case MissionPhase::WAIT_DROPOFF_TASK: return "WAIT_DROPOFF_TASK";
        case MissionPhase::TO_B: return "TO_B";
        case MissionPhase::UNLOAD_DWELL: return "UNLOAD_DWELL";
    }
    return "UNKNOWN";
}

bool sameDepartureCluster(const DepartureClusterCommitment& a,
                          const DepartureClusterCommitment& b) {
    return a.owner_id == b.owner_id &&
           a.owner_path_gen == b.owner_path_gen &&
           a.other_id == b.other_id &&
           a.other_path_gen == b.other_path_gen &&
           a.seed_indices == b.seed_indices &&
           a.cluster_indices == b.cluster_indices &&
           a.waiter_physical_entry_s == b.waiter_physical_entry_s &&
           a.waiter_control_stop_s == b.waiter_control_stop_s &&
           a.owner_release_exit_s == b.owner_release_exit_s &&
           a.other_release_exit_s == b.other_release_exit_s &&
           a.active == b.active &&
           a.handed_off_from_future == b.handed_off_from_future &&
           a.handoff_already_inside == b.handoff_already_inside;
}

void logDepartureCluster(
    const std::function<void(const std::string&)>& sink,
    const char* event, const char* reason,
    const DepartureClusterCommitment& commitment,
    double owner_s, double other_s, double other_speed,
    double max_decel, double dt) {
    if (!sink) return;
    const double overshoot = std::max(
        0.0, other_s - commitment.waiter_control_stop_s);
    const double physical_remaining =
        commitment.waiter_physical_entry_s - other_s;
    const double stopping_distance =
        other_speed * other_speed /
            (2.0 * std::max(1e-6, max_decel)) +
        other_speed * dt;
    const bool braking_feasible =
        commitment.waiter_control_stop_s - other_s + 1e-9 >=
        stopping_distance;
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[DEPARTURE_CLUSTER] event=" << event
         << " reason=" << reason
         << " owner=V" << commitment.owner_id
         << " owner_gen=" << commitment.owner_path_gen
         << " other=V" << commitment.other_id
         << " other_gen=" << commitment.other_path_gen
         << " zones=[";
    for (size_t i = 0; i < commitment.cluster_indices.size(); ++i) {
        if (i > 0) line << ",";
        line << commitment.cluster_indices[i];
    }
    line << "] physical_entry_s="
         << commitment.waiter_physical_entry_s
         << " control_stop_s=" << commitment.waiter_control_stop_s
         << " stop_line_overshoot=" << overshoot
         << " physical_entry_remaining=" << physical_remaining
         << " braking_feasible="
         << (braking_feasible ? "true" : "false")
         << " owner_release_exit_s=" << commitment.owner_release_exit_s
         << " owner_s=" << owner_s
         << " other_s=" << other_s
         << " future_handoff="
         << (commitment.handed_off_from_future ? "true" : "false")
         << " already_inside="
         << (commitment.handoff_already_inside ? "true" : "false");
    sink(line.str());
}

}  // namespace

A1Coordinator::A1Coordinator(const MapParam& map_param,
                             const MultiVehicleConfig& config)
    : mp_(map_param), cfg_(config) {}

void A1Coordinator::resetPlanDiagnostics() {
    decision_logs_.clear();
}

void A1Coordinator::clearFutureCommitment() {
    future_commitment_ = FutureA1Commitment{};
}

void A1Coordinator::setFutureCommitment(
    const FutureA1Commitment& commitment) {
    if (commitment.owner_id != admission_log_owner_id_ ||
        commitment.owner_path_gen != admission_log_owner_path_gen_) {
        admission_logged_.clear();
        admission_log_owner_id_ = commitment.owner_id;
        admission_log_owner_path_gen_ = commitment.owner_path_gen;
    }
    future_commitment_ = commitment;
}

FutureA1Commitment A1Coordinator::selectOwner(
    const std::vector<FutureA1ArrivalCandidate>& candidates,
    double tie_window, const PriorityWinner& priority_winner) const {
    std::vector<FutureA1RankedCandidate> ranked;
    ranked.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        ranked.push_back({candidate.vehicle_id, candidate.arrival_time});
    }
    const int best_id = selectFutureA1Candidate(
        ranked, tie_window, priority_winner);
    FutureA1Commitment result;
    const auto best = std::find_if(
        candidates.begin(), candidates.end(),
        [&](const FutureA1ArrivalCandidate& candidate) {
            return candidate.vehicle_id == best_id;
        });
    if (best == candidates.end()) return result;
    result.owner_id = best->vehicle_id;
    result.owner_path_gen = best->path_gen;
    result.predicted_a1_arrival_time = best->arrival_time;
    result.predicted_to_b_time = best->to_b_time;
    return result;
}

FutureA1Commitment A1Coordinator::retainOwner(
    const std::vector<FutureA1ArrivalCandidate>& candidates,
    const std::vector<VehicleAgent>& vehicles,
    std::string& reason) const {
    reason = "no_service_owner";
    if (!future_commitment_.valid()) return FutureA1Commitment{};
    const auto owner_it = std::find_if(
        vehicles.begin(), vehicles.end(), [&](const VehicleAgent& vehicle) {
            return vehicle.id == future_commitment_.owner_id;
        });
    if (owner_it == vehicles.end()) {
        reason = "owner_missing";
        return FutureA1Commitment{};
    }
    const VehicleAgent& owner = *owner_it;
    FutureA1Commitment retained = future_commitment_;
    auto candidateForOwner = [&]() {
        return std::find_if(
            candidates.begin(), candidates.end(),
            [&](const FutureA1ArrivalCandidate& candidate) {
                return candidate.vehicle_id == owner.id;
            });
    };

    if (owner.mission_phase == MissionPhase::TO_A1) {
        if (owner.mode != VehicleMode::ACTIVE ||
            owner.leg_target != LegTargetKind::A1 ||
            owner.path_gen != retained.owner_path_gen ||
            !owner.pending_dropoff_valid ||
            owner.pending_dropoff_track.empty()) {
            reason = "to_a1_service_invalid";
            return FutureA1Commitment{};
        }
        const auto prediction = candidateForOwner();
        if (prediction != candidates.end()) {
            retained.predicted_a1_arrival_time = prediction->arrival_time;
            retained.predicted_to_b_time = prediction->to_b_time;
        }
        reason = "service_owner_locked_to_a1";
        return retained;
    }

    if (owner.mission_phase == MissionPhase::PICKUP_DWELL) {
        if (owner.mode != VehicleMode::DWELL ||
            owner.path_gen != retained.owner_path_gen ||
            !owner.pending_dropoff_valid ||
            owner.pending_dropoff_track.empty()) {
            reason = "pickup_service_invalid";
            return FutureA1Commitment{};
        }
        retained.predicted_a1_arrival_time = 0.0;
        retained.predicted_to_b_time = std::max(0.0, owner.dwell_remaining);
        reason = "service_owner_locked_pickup";
        return retained;
    }

    if (owner.mission_phase == MissionPhase::UNLOAD_DWELL &&
        owner.path_gen == retained.owner_path_gen &&
        owner.path_s >= owner.a1_departure_priority_until_s - 1e-9 &&
        activeDepartureClusterCount(owner.id) == 0) {
        reason = "departure_resource_clear";
        return FutureA1Commitment{};
    }

    if (owner.mission_phase != MissionPhase::TO_B ||
        owner.mode != VehicleMode::ACTIVE ||
        owner.leg_target != LegTargetKind::B_SLOT || !owner.loaded ||
        owner.a1_departure_priority_until_s <= 1e-9) {
        reason = "service_phase_invalid";
        return FutureA1Commitment{};
    }
    const bool same_generation =
        owner.path_gen == retained.owner_path_gen;
    const bool prepared_handoff =
        owner.path_gen == retained.owner_path_gen + 1;
    if (!same_generation && !prepared_handoff) {
        reason = "unexplained_path_gen_change";
        return FutureA1Commitment{};
    }

    const size_t active_clusters = activeDepartureClusterCount(owner.id);
    const bool prefix_clear =
        owner.path_s >= owner.a1_departure_priority_until_s - 1e-9;
    if (prefix_clear && active_clusters == 0) {
        reason = "departure_resource_clear";
        return FutureA1Commitment{};
    }
    if (!owner.a1_departure_plan_active && !prefix_clear) {
        reason = "departure_plan_lost_before_clear";
        return FutureA1Commitment{};
    }
    retained.owner_path_gen = owner.path_gen;
    retained.predicted_a1_arrival_time = 0.0;
    retained.predicted_to_b_time = 0.0;
    reason = active_clusters > 0
        ? "service_owner_locked_active_cluster"
        : "service_owner_locked_departure_prefix";
    return retained;
}

FutureA1Update A1Coordinator::updateFutureOwner(
    const std::vector<FutureA1ArrivalCandidate>& candidates,
    const std::vector<VehicleAgent>& vehicles, double tie_window,
    const PriorityWinner& priority_winner) {
    FutureA1Update update;
    update.previous = future_commitment_;
    if (future_commitment_.valid()) {
        update.current = retainOwner(candidates, vehicles, update.reason);
    } else {
        update.current = selectOwner(candidates, tie_window, priority_winner);
        update.reason = update.current.valid()
            ? "service_owner_selected" : "no_candidate";
    }
    if (update.current.owner_id != admission_log_owner_id_ ||
        update.current.owner_path_gen != admission_log_owner_path_gen_) {
        admission_logged_.clear();
        admission_log_owner_id_ = update.current.owner_id;
        admission_log_owner_path_gen_ = update.current.owner_path_gen;
    }
    future_commitment_ = update.current;
    return update;
}

A1Coordinator::FutureA1ZoneSelection A1Coordinator::selectProtectedZones(
    const std::vector<ConflictZone>& canonical_zones,
    bool preview_is_lo, double protected_until,
    double other_path_s) const {
    FutureA1ZoneSelection selection;
    selection.normalized_zones.reserve(canonical_zones.size());
    std::vector<FutureA1ConflictInterval> intervals;
    intervals.reserve(canonical_zones.size());
    for (const ConflictZone& canonical : canonical_zones) {
        ConflictZone normalized = canonical;
        if (!preview_is_lo) {
            std::swap(normalized.s_self_enter, normalized.s_other_enter);
            std::swap(normalized.s_self_exit, normalized.s_other_exit);
        }
        selection.normalized_zones.push_back(normalized);
        intervals.push_back(FutureA1ConflictInterval{
            normalized.s_self_enter, normalized.s_self_exit,
            normalized.s_other_enter, normalized.s_other_exit});
    }
    const FutureA1ProtectedCluster cluster = selectFutureA1ProtectedCluster(
        intervals, protected_until, other_path_s);
    selection.seed_indices = cluster.seed_indices;
    selection.protected_indices = cluster.protected_indices;
    selection.other_already_inside = cluster.other_already_inside;
    if (cluster.upstream_other_enter) {
        for (size_t index : selection.protected_indices) {
            if (std::abs(selection.normalized_zones[index].s_other_enter -
                         *cluster.upstream_other_enter) <= 1e-9) {
                selection.upstream_index = static_cast<int>(index);
                break;
            }
        }
    }
    return selection;
}

A1Coordinator::PhysicalProgress A1Coordinator::physicalProgress(
    const VehicleAgent& vehicle) const {
    PhysicalProgress result;
    if (vehicle.track.empty()) return result;
    if (!cfg_.real_mode) {
        result.valid = true;
        result.heading_aligned = true;
        result.path_s = std::max(
            0.0, std::min(vehicle.path_s, vehicle.track.length()));
        return result;
    }
    if (!vehicle.real_pose_valid || !vehicle.real_pose_fresh) return result;

    const double lo = std::max(0.0, vehicle.path_s - 0.15);
    const double hi = std::min(vehicle.track.length(), vehicle.path_s + 0.75);
    double best_s = lo;
    double best_d2 = std::numeric_limits<double>::infinity();
    constexpr double kProjectionStep = 0.01;
    for (double s = lo; s <= hi + 1e-9; s += kProjectionStep) {
        const double query_s = std::min(s, hi);
        const RoughWp pose = vehicle.track.poseAtS(query_s);
        const double d2 =
            (pose.x - vehicle.real_x) * (pose.x - vehicle.real_x) +
            (pose.y - vehicle.real_y) * (pose.y - vehicle.real_y);
        if (d2 < best_d2) {
            best_d2 = d2;
            best_s = query_s;
        }
    }
    const RoughWp track_pose = vehicle.track.poseAtS(best_s);
    result.valid = true;
    result.heading_aligned =
        std::cos(vehicle.real_yaw - track_pose.theta) > 0.5;
    result.path_s = best_s;
    return result;
}

double A1Coordinator::occupancyPathS(const VehicleAgent& vehicle,
                                     bool* confirmed) const {
    const PhysicalProgress progress = physicalProgress(vehicle);
    const bool ok = progress.valid && progress.heading_aligned;
    if (confirmed != nullptr) *confirmed = ok;
    return ok ? progress.path_s : vehicle.path_s;
}

void A1Coordinator::updatePhysicalDepartureState(VehicleAgent& owner) const {
    if (!owner.a1_departure_plan_active ||
        owner.mission_phase != MissionPhase::TO_B) {
        return;
    }
    const PhysicalProgress progress = physicalProgress(owner);
    if (!progress.valid || !progress.heading_aligned) return;
    if (progress.path_s > 1e-3) {
        owner.a1_physical_departure_started = true;
    }
}

A1PairAuthority A1Coordinator::futureAuthorityForPair(
    const VehicleAgent& a, const VehicleAgent& b,
    const ComputeZones& compute_full,
    const ComputeZones& canonical_zones) {
    A1PairAuthority result;
    if (!future_commitment_.valid() ||
        a.mission_phase != MissionPhase::TO_A1 ||
        b.mission_phase != MissionPhase::TO_A1) {
        return result;
    }
    const VehicleAgent* owner = nullptr;
    const VehicleAgent* other = nullptr;
    if (a.id == future_commitment_.owner_id &&
        a.path_gen == future_commitment_.owner_path_gen) {
        owner = &a;
        other = &b;
    } else if (b.id == future_commitment_.owner_id &&
               b.path_gen == future_commitment_.owner_path_gen) {
        owner = &b;
        other = &a;
    }
    if (owner == nullptr || !owner->pending_dropoff_valid ||
        owner->pending_dropoff_track.empty() ||
        owner->a1_departure_priority_until_s <= 1e-9) {
        return result;
    }

    VehicleAgent preview = *owner;
    preview.track = owner->pending_dropoff_track;
    preview.path_s = 0.0;
    preview.path_gen = owner->path_gen + 1;
    preview.mode = VehicleMode::ACTIVE;
    preview.mission_phase = MissionPhase::TO_B;
    const bool preview_is_lo = preview.id < other->id;
    const VehicleAgent& lo = preview_is_lo ? preview : *other;
    const VehicleAgent& hi = preview_is_lo ? *other : preview;
    const std::pair<int, int> key{lo.id, hi.id};
    ConflictCacheEntry& cache = future_conflict_cache_[key];
    if (cache.gen_lo != lo.path_gen || cache.gen_hi != hi.path_gen) {
        cache.blocks = compute_full(lo, hi);
        cache.gen_lo = lo.path_gen;
        cache.gen_hi = hi.path_gen;
    }
    bool other_confirmed = false;
    const double other_s = occupancyPathS(*other, &other_confirmed);
    const FutureA1ZoneSelection selected = selectProtectedZones(
        cache.blocks, preview_is_lo,
        owner->a1_departure_priority_until_s, other_s);
    if (selected.protected_indices.empty() ||
        (other_confirmed && selected.other_already_inside)) {
        return result;
    }

    const bool owner_is_lo = owner->id < other->id;
    const VehicleAgent& ordinary_lo = owner_is_lo ? *owner : *other;
    const VehicleAgent& ordinary_hi = owner_is_lo ? *other : *owner;
    const auto ordinary_blocks = canonical_zones(ordinary_lo, ordinary_hi);
    for (const ConflictZone& canonical : ordinary_blocks) {
        const double owner_exit = owner_is_lo
            ? canonical.s_self_exit : canonical.s_other_exit;
        const double other_enter = owner_is_lo
            ? canonical.s_other_enter : canonical.s_self_enter;
        const double other_exit = owner_is_lo
            ? canonical.s_other_exit : canonical.s_self_exit;
        if (owner->path_s > owner_exit + 1e-9 ||
            other_s > other_exit + 1e-9) {
            continue;
        }
        if (other_confirmed && other_s > other_enter + 1e-9) {
            return result;
        }
    }
    result.protected_pair = true;
    result.owner_id = owner->id;
    result.source = A1AuthoritySource::FUTURE_COMMITMENT;
    return result;
}

A1PairAuthority A1Coordinator::departureAuthorityForPair(
    const VehicleAgent& a, const VehicleAgent& b) const {
    A1PairAuthority result;
    const std::pair<int, int> key{std::min(a.id, b.id), std::max(a.id, b.id)};
    const auto it = departure_clusters_.find(key);
    if (it == departure_clusters_.end() || !it->second.active) return result;
    const DepartureClusterCommitment& cluster = it->second;
    const VehicleAgent* owner = a.id == cluster.owner_id ? &a :
                                b.id == cluster.owner_id ? &b : nullptr;
    const VehicleAgent* other = a.id == cluster.other_id ? &a :
                                b.id == cluster.other_id ? &b : nullptr;
    if (owner == nullptr || other == nullptr ||
        owner->path_gen != cluster.owner_path_gen ||
        other->path_gen != cluster.other_path_gen) {
        return result;
    }
    bool confirmed = false;
    const double other_s = occupancyPathS(*other, &confirmed);
    result.protected_pair = true;
    if (confirmed && futureA1OtherInsideCluster(cluster.intervals, other_s)) {
        result.owner_id = other->id;
        result.source = A1AuthoritySource::ACTUAL_OCCUPANCY;
    } else {
        result.owner_id = owner->id;
        result.source = A1AuthoritySource::DEPARTURE_CLUSTER;
    }
    return result;
}

void A1Coordinator::refreshDepartureClusters(
    std::vector<VehicleAgent>& vehicles,
    const ComputeZones& canonical_zones) {
    auto agentById = [&](int id) -> VehicleAgent* {
        for (VehicleAgent& vehicle : vehicles) {
            if (vehicle.id == id) return &vehicle;
        }
        return nullptr;
    };
    auto eraseWithEvent = [&](auto it, const char* event,
                              const char* reason) {
        VehicleAgent* owner = agentById(it->second.owner_id);
        VehicleAgent* other = agentById(it->second.other_id);
        logDepartureCluster(
            coord_log_sink_, event, reason, it->second,
            owner ? owner->path_s : -1.0,
            other ? other->path_s : -1.0,
            other ? std::max(0.0, other->current_speed) : 0.0,
            cfg_.max_decel, 0.0);
        return departure_clusters_.erase(it);
    };

    for (auto it = departure_clusters_.begin();
         it != departure_clusters_.end();) {
        DepartureClusterCommitment& cluster = it->second;
        VehicleAgent* owner = agentById(cluster.owner_id);
        VehicleAgent* other = agentById(cluster.other_id);
        if (owner == nullptr || other == nullptr || !owner->active() ||
            !other->active() || owner->track.empty() || other->track.empty()) {
            it = eraseWithEvent(it, "INVALIDATE", "vehicle_or_path_invalid");
            continue;
        }
        updatePhysicalDepartureState(*owner);
        if (!cluster.active) {
            if (owner->mission_phase == MissionPhase::TO_B &&
                owner->path_gen == cluster.owner_path_gen &&
                other->path_gen == cluster.other_path_gen &&
                other->mission_phase == MissionPhase::TO_A1) {
                cluster.active = true;
                bool confirmed = false;
                const double other_s = occupancyPathS(*other, &confirmed);
                cluster.handoff_already_inside = confirmed &&
                    futureA1OtherInsideCluster(cluster.intervals, other_s);
                logDepartureCluster(
                    coord_log_sink_, "CREATE",
                    cluster.handoff_already_inside
                        ? "handoff_already_inside" : "future_handoff",
                    cluster, owner->path_s, other_s,
                    std::max(0.0, other->current_speed), cfg_.max_decel, 0.0);
                ++it;
                continue;
            }
            const bool preview_still_matches =
                (owner->mission_phase == MissionPhase::TO_A1 ||
                 owner->mission_phase == MissionPhase::PICKUP_DWELL) &&
                owner->pending_dropoff_valid &&
                !owner->pending_dropoff_track.empty() &&
                owner->path_gen + 1 == cluster.owner_path_gen &&
                other->path_gen == cluster.other_path_gen &&
                other->mission_phase == MissionPhase::TO_A1 &&
                (owner->mission_phase == MissionPhase::PICKUP_DWELL ||
                 (future_commitment_.valid() &&
                  future_commitment_.owner_id == owner->id &&
                  future_commitment_.owner_path_gen == owner->path_gen));
            if (!preview_still_matches) {
                it = eraseWithEvent(it, "INVALIDATE", "staged_handoff_invalid");
            } else {
                ++it;
            }
            continue;
        }

        if (!departureClusterGenerationsMatch(
                cluster.owner_path_gen, owner->path_gen,
                cluster.other_path_gen, other->path_gen)) {
            it = eraseWithEvent(it, "INVALIDATE", "path_gen_changed");
        } else if (owner->mission_phase != MissionPhase::TO_B ||
                   other->mission_phase != MissionPhase::TO_A1) {
            it = eraseWithEvent(it, "INVALIDATE", "mission_phase_changed");
        } else {
            bool owner_confirmed = false;
            bool other_confirmed = false;
            const double owner_s = occupancyPathS(*owner, &owner_confirmed);
            const double other_s = occupancyPathS(*other, &other_confirmed);
            const bool owner_cleared = owner_confirmed &&
                (!cfg_.real_mode || owner->a1_physical_departure_started) &&
                owner_s > cluster.owner_release_exit_s + 1e-9;
            const bool other_cleared = other_confirmed &&
                other_s > cluster.other_release_exit_s + 1e-9;
            if (owner_cleared || other_cleared) {
                it = eraseWithEvent(
                    it, "RELEASE",
                    owner_cleared ? "owner_physically_cleared_cluster"
                                  : "other_physically_cleared_cluster");
            } else {
                ++it;
            }
        }
    }

    for (VehicleAgent& owner : vehicles) {
        if (!owner.active() || owner.track.empty() ||
            owner.mission_phase != MissionPhase::TO_B ||
            !owner.a1_departure_plan_active ||
            owner.a1_departure_priority_until_s <= 1e-9) {
            continue;
        }
        for (VehicleAgent& other : vehicles) {
            if (other.id == owner.id || !other.active() || other.track.empty() ||
                other.mission_phase != MissionPhase::TO_A1) {
                continue;
            }
            const std::pair<int, int> key{std::min(owner.id, other.id),
                                          std::max(owner.id, other.id)};
            if (departure_clusters_.count(key) != 0) continue;
            const bool owner_is_lo = owner.id < other.id;
            const VehicleAgent& lo = owner_is_lo ? owner : other;
            const VehicleAgent& hi = owner_is_lo ? other : owner;
            const auto blocks = canonical_zones(lo, hi);
            bool other_confirmed = false;
            const double other_s = occupancyPathS(other, &other_confirmed);
            const FutureA1ZoneSelection selected = selectProtectedZones(
                blocks, owner_is_lo,
                owner.a1_departure_priority_until_s, other_s);
            if (selected.protected_indices.empty() ||
                selected.upstream_index < 0) {
                continue;
            }
            DepartureClusterCommitment cluster;
            cluster.owner_id = owner.id;
            cluster.owner_path_gen = owner.path_gen;
            cluster.other_id = other.id;
            cluster.other_path_gen = other.path_gen;
            cluster.seed_indices = selected.seed_indices;
            cluster.cluster_indices = selected.protected_indices;
            cluster.waiter_physical_entry_s =
                selected.normalized_zones[static_cast<size_t>(
                    selected.upstream_index)].s_other_enter;
            cluster.waiter_control_stop_s = std::max(
                0.0, cluster.waiter_physical_entry_s -
                         cfg_.a1_control_stop_margin);
            cluster.active = true;
            cluster.handoff_already_inside =
                other_confirmed && selected.other_already_inside;
            for (size_t index : selected.protected_indices) {
                const ConflictZone& zone = selected.normalized_zones[index];
                cluster.intervals.push_back(FutureA1ConflictInterval{
                    zone.s_self_enter, zone.s_self_exit,
                    zone.s_other_enter, zone.s_other_exit});
                cluster.owner_release_exit_s = std::max(
                    cluster.owner_release_exit_s, zone.s_self_exit);
                cluster.other_release_exit_s = std::max(
                    cluster.other_release_exit_s, zone.s_other_exit);
            }
            departure_clusters_[key] = cluster;
            logDepartureCluster(
                coord_log_sink_, "CREATE",
                cluster.handoff_already_inside
                    ? "handoff_already_inside" : "deterministic_rebuild",
                cluster, owner.path_s, other_s,
                std::max(0.0, other.current_speed), cfg_.max_decel, 0.0);
        }
    }
}

std::vector<A1ActionRequest> A1Coordinator::enforceFutureAdmission(
    std::vector<VehicleAgent>& vehicles, double dt,
    const ComputeZones& compute_full,
    const ComputeZones& canonical_zones) {
    std::vector<A1ActionRequest> requests;
    if (!future_commitment_.valid()) return requests;
    VehicleAgent* owner = nullptr;
    for (VehicleAgent& vehicle : vehicles) {
        if (vehicle.id == future_commitment_.owner_id) {
            owner = &vehicle;
            break;
        }
    }
    if (owner == nullptr || owner->path_gen != future_commitment_.owner_path_gen ||
        !owner->pending_dropoff_valid || owner->pending_dropoff_track.empty() ||
        (owner->mission_phase != MissionPhase::TO_A1 &&
         owner->mission_phase != MissionPhase::PICKUP_DWELL)) {
        return requests;
    }
    VehicleAgent preview = *owner;
    preview.track = owner->pending_dropoff_track;
    preview.path_s = 0.0;
    preview.path_gen = owner->path_gen + 1;
    preview.mode = VehicleMode::ACTIVE;
    preview.mission_phase = MissionPhase::TO_B;
    const double protected_until = owner->a1_departure_priority_until_s;
    if (protected_until <= 1e-9) return requests;

    for (VehicleAgent& other : vehicles) {
        if (other.id == owner->id || !other.active() ||
            other.mission_phase != MissionPhase::TO_A1 || other.track.empty()) {
            continue;
        }
        bool other_confirmed = false;
        const double other_s = occupancyPathS(other, &other_confirmed);
        const bool preview_is_lo = preview.id < other.id;
        const VehicleAgent& lo = preview_is_lo ? preview : other;
        const VehicleAgent& hi = preview_is_lo ? other : preview;
        const std::pair<int, int> cache_key{lo.id, hi.id};
        ConflictCacheEntry& cache = future_conflict_cache_[cache_key];
        if (cache.gen_lo != lo.path_gen || cache.gen_hi != hi.path_gen) {
            cache.blocks = compute_full(lo, hi);
            cache.gen_lo = lo.path_gen;
            cache.gen_hi = hi.path_gen;
        }
        const FutureA1ZoneSelection future_zones = selectProtectedZones(
            cache.blocks, preview_is_lo, protected_until, other_s);
        std::optional<double> future_entry_s;
        ConflictZone future_selected;
        if (future_zones.upstream_index >= 0) {
            future_selected = future_zones.normalized_zones[
                static_cast<size_t>(future_zones.upstream_index)];
            future_entry_s = future_selected.s_other_enter;
        }

        bool ordinary_already_inside = false;
        std::optional<double> ordinary_entry_s;
        ConflictZone ordinary_selected;
        const bool owner_is_lo = owner->id < other.id;
        const VehicleAgent& ordinary_lo = owner_is_lo ? *owner : other;
        const VehicleAgent& ordinary_hi = owner_is_lo ? other : *owner;
        const auto ordinary_blocks = canonical_zones(ordinary_lo, ordinary_hi);
        for (const ConflictZone& canonical : ordinary_blocks) {
            ConflictZone zone = canonical;
            if (!owner_is_lo) {
                std::swap(zone.s_self_enter, zone.s_other_enter);
                std::swap(zone.s_self_exit, zone.s_other_exit);
            }
            if (owner->path_s > zone.s_self_exit + 1e-9 ||
                other_s > zone.s_other_exit + 1e-9) {
                continue;
            }
            if (other_confirmed && other_s > zone.s_other_enter + 1e-9) {
                ordinary_already_inside = true;
                ordinary_entry_s = zone.s_other_enter;
                ordinary_selected = zone;
                break;
            }
            if (!ordinary_entry_s || zone.s_other_enter < *ordinary_entry_s) {
                ordinary_entry_s = zone.s_other_enter;
                ordinary_selected = zone;
            }
        }
        if (!future_entry_s) continue;

        const bool already_inside = other_confirmed &&
            (future_zones.other_already_inside || ordinary_already_inside);
        const std::optional<double> physical_entry_s = futureA1StopBoundary(
            future_entry_s, ordinary_entry_s);
        const double control_stop_s = std::max(
            0.0, *physical_entry_s - cfg_.a1_control_stop_margin);
        const bool ordinary_selected_boundary = ordinary_entry_s &&
            std::abs(*ordinary_entry_s - *physical_entry_s) <= 1e-9;
        const ConflictZone& selected = ordinary_selected_boundary
            ? ordinary_selected : future_selected;

        const std::pair<int, int> cluster_key{
            std::min(owner->id, other.id), std::max(owner->id, other.id)};
        auto existing = departure_clusters_.find(cluster_key);
        if (existing == departure_clusters_.end() || !existing->second.active) {
            DepartureClusterCommitment staged;
            staged.owner_id = owner->id;
            staged.owner_path_gen = preview.path_gen;
            staged.other_id = other.id;
            staged.other_path_gen = other.path_gen;
            staged.seed_indices = future_zones.seed_indices;
            staged.cluster_indices = future_zones.protected_indices;
            staged.waiter_physical_entry_s = *physical_entry_s;
            staged.waiter_control_stop_s = control_stop_s;
            staged.handed_off_from_future = true;
            staged.handoff_already_inside = already_inside;
            for (size_t index : future_zones.protected_indices) {
                const ConflictZone& zone = future_zones.normalized_zones[index];
                staged.intervals.push_back(FutureA1ConflictInterval{
                    zone.s_self_enter, zone.s_self_exit,
                    zone.s_other_enter, zone.s_other_exit});
                staged.owner_release_exit_s = std::max(
                    staged.owner_release_exit_s, zone.s_self_exit);
                staged.other_release_exit_s = std::max(
                    staged.other_release_exit_s, zone.s_other_exit);
            }
            departure_clusters_[cluster_key] = std::move(staged);
        }

        const double speed = std::max(0.0, other.current_speed);
        const double stopping_distance =
            speed * speed / (2.0 * std::max(1e-6, cfg_.max_decel)) +
            speed * dt;
        const double distance = control_stop_s - other_s;
        const double overshoot = std::max(0.0, other_s - control_stop_s);
        const double physical_remaining = *physical_entry_s - other_s;
        const bool braking_feasible = distance + 1e-9 >= stopping_distance;
        auto appendGeometry = [&](std::ostringstream& line) {
            line << " physical_entry_s=" << *physical_entry_s
                 << " control_stop_s=" << control_stop_s
                 << " stop_line_overshoot=" << overshoot
                 << " physical_entry_remaining=" << physical_remaining
                 << " stopping_distance=" << stopping_distance
                 << " braking_feasible="
                 << (braking_feasible ? "true" : "false")
                 << " other_s=" << other_s
                 << " pose_confirmed="
                 << (other_confirmed ? "true" : "false")
                 << " already_inside="
                 << (already_inside ? "true" : "false")
                 << " seed_zones=[";
            for (size_t i = 0; i < future_zones.seed_indices.size(); ++i) {
                if (i > 0) line << ",";
                line << future_zones.seed_indices[i];
            }
            line << "] closure_zones=[";
            for (size_t i = 0; i < future_zones.protected_indices.size(); ++i) {
                if (i > 0) line << ",";
                line << future_zones.protected_indices[i];
            }
            line << "]";
        };

        const std::pair<int, int> log_key{owner->id, other.id};
        if (already_inside) {
            if (admission_logged_.insert(log_key).second) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(3)
                     << "[FUTURE_A1_ADMISSION] owner=V" << owner->id
                     << " blocked=V" << other.id
                     << " reason=actual_occupied_priority early_stop=false"
                     << " holder=V" << other.id
                     << " conflict_zone=(" << selected.x << ","
                     << selected.y << ")";
                appendGeometry(line);
                if (coord_log_sink_) coord_log_sink_(line.str());
                ROS_WARN("%s %s", debug_log_prefix_.c_str(),
                         line.str().c_str());
            }
            continue;
        }
        const bool braking_required_now =
            distance <= stopping_distance + 1e-9;
        if (!braking_required_now) continue;
        requests.push_back(A1ActionRequest{
            other.id, VehicleAction::STOP, owner->id,
            "future_a1_exit_priority"});
        if (admission_logged_.insert(log_key).second) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[FUTURE_A1_ADMISSION] owner=V" << owner->id
                 << " blocked=V" << other.id
                 << " reason=future_a1_exit_priority early_stop=true"
                 << " braking_required_now=true holder=V" << owner->id
                 << " conflict_zone=(" << selected.x << ","
                 << selected.y << ")";
            appendGeometry(line);
            if (coord_log_sink_) coord_log_sink_(line.str());
            ROS_WARN("%s %s", debug_log_prefix_.c_str(),
                     line.str().c_str());
        }
    }
    return requests;
}

std::vector<A1ActionRequest> A1Coordinator::enforceDepartureClusters(
    std::vector<VehicleAgent>& vehicles, double dt) {
    std::vector<A1ActionRequest> requests;
    auto agentById = [&](int id) -> VehicleAgent* {
        for (VehicleAgent& vehicle : vehicles) {
            if (vehicle.id == id) return &vehicle;
        }
        return nullptr;
    };
    for (auto& entry : departure_clusters_) {
        DepartureClusterCommitment& cluster = entry.second;
        if (!cluster.active) continue;
        VehicleAgent* owner = agentById(cluster.owner_id);
        VehicleAgent* other = agentById(cluster.other_id);
        if (owner == nullptr || other == nullptr) continue;
        bool confirmed = false;
        const double other_s = occupancyPathS(*other, &confirmed);
        const bool already_inside = confirmed &&
            futureA1OtherInsideCluster(cluster.intervals, other_s);
        if (already_inside) continue;
        const double distance = cluster.waiter_control_stop_s - other_s;
        const double speed = std::max(0.0, other->current_speed);
        const double stopping_distance =
            speed * speed / (2.0 * std::max(1e-6, cfg_.max_decel)) +
            speed * dt;
        const bool braking_required_now =
            distance <= stopping_distance + 1e-9;
        if (!braking_required_now) continue;
        requests.push_back(A1ActionRequest{
            other->id, VehicleAction::STOP, owner->id,
            "departure_cluster_priority"});
        if (!cluster.hold_logged) {
            logDepartureCluster(
                coord_log_sink_, "HOLD", "cluster_control_stop",
                cluster, owner->path_s, other_s, speed,
                cfg_.max_decel, dt);
            cluster.hold_logged = true;
        }
    }
    return requests;
}

A1LaunchAdmission A1Coordinator::checkLaunchAdmission(
    const VehicleAgent& service_owner,
    const VehicleAgent& launch_candidate,
    const ComputeZones& compute_full) const {
    A1LaunchAdmission result;
    if (!launch_candidate.active() || launch_candidate.track.empty() ||
        launch_candidate.mission_phase != MissionPhase::TO_A1 ||
        service_owner.id == launch_candidate.id ||
        service_owner.a1_departure_priority_until_s <= 1e-9) {
        return result;
    }
    VehicleAgent exit = service_owner;
    if ((service_owner.mission_phase == MissionPhase::TO_A1 ||
         service_owner.mission_phase == MissionPhase::PICKUP_DWELL) &&
        service_owner.pending_dropoff_valid &&
        !service_owner.pending_dropoff_track.empty()) {
        exit.track = service_owner.pending_dropoff_track;
        exit.path_s = 0.0;
        exit.path_gen = service_owner.path_gen + 1;
        exit.mode = VehicleMode::ACTIVE;
        exit.mission_phase = MissionPhase::TO_B;
        result.owner_uses_pending_preview = true;
    } else if (service_owner.active() &&
               service_owner.mission_phase == MissionPhase::TO_B &&
               service_owner.a1_departure_plan_active &&
               !service_owner.track.empty() &&
               service_owner.path_s <
                   service_owner.a1_departure_priority_until_s - 1e-9) {
        // Use the active TO_B path while its protected prefix remains live.
    } else {
        return result;
    }
    const bool exit_is_lo = exit.id < launch_candidate.id;
    const VehicleAgent& lo = exit_is_lo ? exit : launch_candidate;
    const VehicleAgent& hi = exit_is_lo ? launch_candidate : exit;
    const auto blocks = compute_full(lo, hi);
    bool candidate_confirmed = false;
    const double candidate_s = occupancyPathS(
        launch_candidate, &candidate_confirmed);
    const FutureA1ZoneSelection selected = selectProtectedZones(
        blocks, exit_is_lo, service_owner.a1_departure_priority_until_s,
        candidate_s);
    const bool candidate_has_cleared_slot =
        candidate_s + 1e-9 >= launch_candidate.slot_departure_clear_s;
    result.actual_occupancy_priority = candidate_confirmed &&
        selected.other_already_inside && candidate_has_cleared_slot;
    if (result.actual_occupancy_priority) return result;
    for (size_t index : selected.protected_indices) {
        const ConflictZone& zone = selected.normalized_zones[index];
        if (zone.s_other_enter >
            launch_candidate.slot_departure_clear_s + 1e-9) {
            continue;
        }
        if (!result.owner_uses_pending_preview &&
            service_owner.path_s > zone.s_self_exit + 1e-9) {
            continue;
        }
        ++result.protected_zone_count;
    }
    result.departure_resource_conflict = result.protected_zone_count > 0;
    return result;
}

size_t A1Coordinator::activeDepartureClusterCount(int owner_id) const {
    size_t count = 0;
    for (const auto& entry : departure_clusters_) {
        if (entry.second.active && entry.second.owner_id == owner_id) ++count;
    }
    return count;
}

bool A1Coordinator::shouldLogDecision(const VehicleAgent& vehicle,
                                      int blocker_id) {
    return decision_logs_.insert(std::make_tuple(
        vehicle.id, blocker_id, static_cast<int>(vehicle.mission_phase),
        static_cast<int>(vehicle.requested_action))).second;
}

void A1Coordinator::logDecision(const VehicleAgent& vehicle,
                                const VehicleAgent* blocker,
                                int blocker_id) const {
    if (!coord_log_sink_ || vehicle.track.empty()) return;
    const RoughWp pose = vehicle.track.poseAtS(vehicle.path_s);
    const double distance_to_a1 = std::hypot(
        pose.x - cfg_.a1_pickup_center_x,
        pose.y - cfg_.a1_pickup_center_y);
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[A1_DECISION] V" << vehicle.id
         << " action=" << actionName(vehicle.requested_action)
         << " reason=" << vehicle.reason
         << " blocker=V" << blocker_id
         << " phase=" << missionPhaseName(vehicle.mission_phase)
         << " dist_to_A1=" << distance_to_a1
         << " blocker_phase="
         << (blocker ? missionPhaseName(blocker->mission_phase) : "UNKNOWN");
    coord_log_sink_(line.str());
}

A1CoordinationSnapshot A1Coordinator::snapshot() const {
    A1CoordinationSnapshot result;
    result.departure_clusters = departure_clusters_;
    return result;
}

void A1Coordinator::restore(const A1CoordinationSnapshot& snapshot) {
    for (const auto& current : departure_clusters_) {
        const auto incoming = snapshot.departure_clusters.find(current.first);
        if (current.second.active &&
            (incoming == snapshot.departure_clusters.end() ||
             !incoming->second.active)) {
            logDepartureCluster(
                coord_log_sink_, "RELEASE", "snapshot_restore",
                current.second, -1.0, -1.0, 0.0, cfg_.max_decel, 0.0);
        }
    }
    for (const auto& incoming : snapshot.departure_clusters) {
        const auto current = departure_clusters_.find(incoming.first);
        if (incoming.second.active &&
            (current == departure_clusters_.end() ||
             !current->second.active)) {
            logDepartureCluster(
                coord_log_sink_, "CREATE", "snapshot_restore",
                incoming.second, -1.0, -1.0, 0.0, cfg_.max_decel, 0.0);
        } else if (incoming.second.active &&
                   current != departure_clusters_.end() &&
                   !sameDepartureCluster(current->second, incoming.second)) {
            logDepartureCluster(
                coord_log_sink_, "HOLD", "snapshot_restore",
                incoming.second, -1.0, -1.0, 0.0, cfg_.max_decel, 0.0);
        }
    }
    departure_clusters_ = snapshot.departure_clusters;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
