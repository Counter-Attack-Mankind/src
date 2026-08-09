#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace forklift_planner {
namespace multi_vehicle {

struct FutureA1RankedCandidate {
    int vehicle_id = -1;
    double arrival_time = -1.0;
};

inline bool futureA1ArrivalWithinHorizon(double arrival_time,
                                         double horizon) {
    return std::isfinite(arrival_time) && arrival_time >= 0.0 &&
           arrival_time <= horizon + 1e-9;
}

inline int selectFutureA1Candidate(
    const std::vector<FutureA1RankedCandidate>& candidates,
    double tie_window,
    const std::function<int(int, int)>& priority_winner) {
    const FutureA1RankedCandidate* best = nullptr;
    for (const FutureA1RankedCandidate& candidate : candidates) {
        if (candidate.vehicle_id < 0 || candidate.arrival_time < 0.0) continue;
        if (best == nullptr ||
            candidate.arrival_time < best->arrival_time - tie_window) {
            best = &candidate;
            continue;
        }
        if (candidate.arrival_time > best->arrival_time + tie_window) continue;

        const int priority = priority_winner
            ? priority_winner(candidate.vehicle_id, best->vehicle_id)
            : -1;
        if (priority == candidate.vehicle_id) {
            best = &candidate;
        } else if (priority != best->vehicle_id &&
                   candidate.vehicle_id < best->vehicle_id) {
            best = &candidate;
        }
    }
    return best == nullptr ? -1 : best->vehicle_id;
}

inline std::optional<double> futureA1StopBoundary(
    const std::optional<double>& future_exit_enter_s,
    const std::optional<double>& ordinary_enter_s) {
    if (future_exit_enter_s && ordinary_enter_s) {
        return std::min(*future_exit_enter_s, *ordinary_enter_s);
    }
    if (future_exit_enter_s) return future_exit_enter_s;
    return ordinary_enter_s;
}

inline std::optional<double> futureA1StopS(
    const std::optional<double>& future_exit_enter_s,
    const std::optional<double>& ordinary_enter_s,
    double stop_buffer) {
    const std::optional<double> boundary =
        futureA1StopBoundary(future_exit_enter_s, ordinary_enter_s);
    if (!boundary) return std::nullopt;
    return std::max(0.0, *boundary - std::max(0.0, stop_buffer));
}

inline std::string futureA1TransitionReason(
    bool previous_valid, int previous_owner_id,
    bool current_valid, int current_owner_id,
    bool owner_locked, bool previous_owner_valid,
    bool previous_owner_horizon_exceeded, bool departure_handoff) {
    if (owner_locked) return "owner_entered_a1_locked";
    if (!previous_valid) {
        return current_valid ? "earliest_candidate" : "no_candidate";
    }
    if (departure_handoff) return "handoff_to_a1_departure_priority";
    if (!previous_owner_valid) return "owner_invalid";
    if (previous_owner_horizon_exceeded) return "owner_horizon_exceeded";
    if (!current_valid) return "no_candidate";
    if (current_owner_id != previous_owner_id) return "earlier_candidate";
    return "earliest_candidate";
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
