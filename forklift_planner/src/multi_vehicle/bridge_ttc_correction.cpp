#include "forklift_planner/multi_vehicle/bridge_ttc_correction.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace forklift_planner {
namespace multi_vehicle {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

struct NearestMatch {
    double s = 0.0;
    double distance = std::numeric_limits<double>::infinity();
};

struct NearestCursor {
    const PathTrack* track = nullptr;
    std::size_t index = 0;
};

enum class RelationState {
    VALID,
    DISTANCE_LOST,
    DIRECTION_LOST,
    INVALID_GEOMETRY,
};

double squaredDistance(const RoughWp& a, const RoughWp& b,
                       int& evaluations) {
    ++evaluations;
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

NearestCursor makeNearestCursor(const PathTrack& track, double seed_s) {
    NearestCursor cursor;
    cursor.track = &track;
    const auto& path = track.path();
    const auto& cumulative_s = track.cumulative_s();
    auto it = std::lower_bound(cumulative_s.begin(), cumulative_s.end(),
                               std::max(0.0, std::min(seed_s, track.length())));
    std::size_t index = it == cumulative_s.end()
        ? path.size() - 1
        : static_cast<std::size_t>(std::distance(cumulative_s.begin(), it));
    cursor.index = index;
    return cursor;
}

NearestMatch projectToSegment(const RoughWp& query,
                              const PathTrack& track,
                              std::size_t from, std::size_t to,
                              int& evaluations) {
    const RoughWp& a = track.path()[from];
    const RoughWp& b = track.path()[to];
    ++evaluations;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double ratio = length_sq <= kEpsilon
        ? 0.0
        : std::max(0.0, std::min(1.0,
              ((query.x - a.x) * dx + (query.y - a.y) * dy) / length_sq));
    const double x = a.x + ratio * dx;
    const double y = a.y + ratio * dy;
    const double distance = std::hypot(query.x - x, query.y - y);
    const auto& cumulative_s = track.cumulative_s();
    return NearestMatch{
        cumulative_s[from] + ratio *
            (cumulative_s[to] - cumulative_s[from]),
        distance};
}

NearestMatch nearestOnPathLocal(const RoughWp& query, NearestCursor& cursor,
                                int& evaluations) {
    const auto& path = cursor.track->path();
    double best_sq = squaredDistance(query, path[cursor.index], evaluations);
    for (;;) {
        std::size_t best_index = cursor.index;
        double candidate_sq = best_sq;
        // A two-waypoint local window can cross a duplicated/near-duplicated
        // cusp without opening a full-path search. The cursor still moves only
        // through adjacent path order, so an overlapping remote traversal
        // cannot cause a discontinuous s jump.
        const std::size_t first = cursor.index > 2 ? cursor.index - 2 : 0;
        const std::size_t last = std::min(
            path.size() - 1, cursor.index + 2);
        for (std::size_t candidate = first; candidate <= last; ++candidate) {
            if (candidate == cursor.index) continue;
            const double value = squaredDistance(
                query, path[candidate], evaluations);
            if (value + kEpsilon < candidate_sq) {
                candidate_sq = value;
                best_index = candidate;
            }
        }
        if (best_index == cursor.index) break;
        cursor.index = best_index;
        best_sq = candidate_sq;
    }

    const auto& cumulative_s = cursor.track->cumulative_s();
    NearestMatch best{cumulative_s[cursor.index], std::sqrt(best_sq)};
    auto consider = [&](std::size_t from, std::size_t to) {
        const NearestMatch candidate = projectToSegment(
            query, *cursor.track, from, to, evaluations);
        if (candidate.distance + kEpsilon < best.distance) best = candidate;
    };
    // Include the immediately adjacent segments on both sides of a possible
    // duplicated cusp waypoint. This is still a constant local window and is
    // needed because equal-distance duplicate vertices cannot win the strict
    // hill-climb comparison above.
    const std::size_t first_segment = cursor.index > 2
        ? cursor.index - 2 : 0;
    const std::size_t last_segment = std::min(
        path.size() - 2, cursor.index + 2);
    for (std::size_t from = first_segment; from <= last_segment; ++from) {
        consider(from, from + 1);
    }
    return best;
}

double motionHeading(const PathTrack& track, double s) {
    // Production path generation stores body heading along FORWARD motion and
    // body heading = motion + pi along REVERSE motion. Keep that established
    // representation here; WpType selects motion heading but does not delimit
    // bridge backtracking.
    const RoughWp pose = track.poseAtS(s);
    return pose.theta +
        (track.typeAtS(s) == WpType::REVERSE ? kPi : 0.0);
}

VehicleBridgeTtcCorrection evaluateVehicle(
    const VehicleAgent& self, const VehicleAgent& other,
    const std::vector<PredictedKinematicSample>& self_prediction,
    double collision_s, double other_collision_s, double original_ttc,
    const MapParam& map_param, const MultiVehicleConfig& config) {
    VehicleBridgeTtcCorrection result;
    result.collision_s = collision_s;
    result.near_boundary_s = collision_s;
    result.original_ttc = original_ttc;
    result.corrected_ttc = original_ttc;
    result.collision_type = self.track.typeAtS(collision_s);
    result.boundary_type = result.collision_type;
    if (self.track.empty() || other.track.empty() ||
        self_prediction.empty()) {
        result.backtrack_end_reason =
            BridgeBacktrackEndReason::INVALID_GEOMETRY;
        return result;
    }

    const double proximity_limit =
        map_param.vehicle_width + config.conflict_margin;
    NearestCursor other_cursor = makeNearestCursor(
        other.track, other_collision_s);

    auto relationAt = [&](double self_s, double& match_s,
                          double& direction_dot, double& distance) {
        const RoughWp self_pose = self.track.poseAtS(self_s);
        const NearestMatch match = nearestOnPathLocal(
            self_pose, other_cursor, result.nearest_search_evaluations);
        match_s = match.s;
        distance = match.distance;
        direction_dot = std::cos(
            motionHeading(self.track, self_s) -
            motionHeading(other.track, match.s));
        if (!std::isfinite(match.s) || !std::isfinite(match.distance) ||
            !std::isfinite(direction_dot)) {
            return RelationState::INVALID_GEOMETRY;
        }
        if (match.distance > proximity_limit + kEpsilon) {
            return RelationState::DISTANCE_LOST;
        }
        if (direction_dot >= config.bridge_opposing_threshold) {
            return RelationState::DIRECTION_LOST;
        }
        return RelationState::VALID;
    };

    auto endReason = [](RelationState state) {
        switch (state) {
            case RelationState::DISTANCE_LOST:
                return BridgeBacktrackEndReason::RELATION_DISTANCE_LOST;
            case RelationState::DIRECTION_LOST:
                return BridgeBacktrackEndReason::RELATION_DIRECTION_LOST;
            case RelationState::INVALID_GEOMETRY:
                return BridgeBacktrackEndReason::INVALID_GEOMETRY;
            case RelationState::VALID:
                break;
        }
        return BridgeBacktrackEndReason::NOT_EVALUATED;
    };

    ++result.backtrack_samples;
    const RelationState collision_relation = relationAt(
        collision_s, result.matched_other_s,
        result.collision_direction_dot, result.collision_match_distance);
    result.end_query_s = collision_s;
    result.end_matched_other_s = result.matched_other_s;
    result.end_match_distance = result.collision_match_distance;
    result.end_direction_dot = result.collision_direction_dot;
    if (collision_relation != RelationState::VALID) {
        result.backtrack_end_reason = endReason(collision_relation);
        return result;
    }
    result.bridge_related = true;
    WpType previous_self_type = result.collision_type;
    WpType previous_other_type = other.track.typeAtS(
        result.matched_other_s);

    const double step = std::max(0.005, config.bridge_backtrack_step);
    double cursor_s = collision_s;
    if (cursor_s <= kEpsilon) {
        result.backtrack_end_reason =
            BridgeBacktrackEndReason::PATH_START;
    }
    while (cursor_s > 0.0 + kEpsilon) {
        const double query_s = std::max(0.0, cursor_s - step);
        const WpType query_self_type = self.track.typeAtS(query_s);
        if (query_self_type != previous_self_type) {
            ++result.self_traversal_changes;
            previous_self_type = query_self_type;
        }
        double match_s = 0.0;
        double direction_dot = 1.0;
        double distance = std::numeric_limits<double>::infinity();
        ++result.backtrack_samples;
        const RelationState relation = relationAt(
            query_s, match_s, direction_dot, distance);
        result.end_query_s = query_s;
        result.end_matched_other_s = match_s;
        result.end_match_distance = distance;
        result.end_direction_dot = direction_dot;
        if (std::isfinite(match_s)) {
            const WpType other_type = other.track.typeAtS(match_s);
            if (other_type != previous_other_type) {
                ++result.nearest_other_traversal_changes;
                previous_other_type = other_type;
            }
        }
        if (relation != RelationState::VALID) {
            result.backtrack_end_reason = endReason(relation);
            break;
        }
        result.near_boundary_s = query_s;
        cursor_s = query_s;
        if (query_s <= kEpsilon) {
            result.backtrack_end_reason =
                BridgeBacktrackEndReason::PATH_START;
            break;
        }
    }
    result.boundary_type = self.track.typeAtS(result.near_boundary_s);

    const double boundary_ttc = predictionTimeAtS(
        self_prediction, result.near_boundary_s);
    if (!std::isfinite(boundary_ttc)) {
        result.backtrack_end_reason =
            BridgeBacktrackEndReason::INVALID_GEOMETRY;
        return result;
    }
    result.corrected_ttc = std::min(original_ttc, boundary_ttc);
    return result;
}

}  // namespace

const char* bridgeBacktrackEndReasonName(BridgeBacktrackEndReason reason) {
    switch (reason) {
        case BridgeBacktrackEndReason::NOT_EVALUATED:
            return "not_evaluated";
        case BridgeBacktrackEndReason::RELATION_DISTANCE_LOST:
            return "relation_distance_lost";
        case BridgeBacktrackEndReason::RELATION_DIRECTION_LOST:
            return "relation_direction_lost";
        case BridgeBacktrackEndReason::PATH_START:
            return "path_start";
        case BridgeBacktrackEndReason::INVALID_GEOMETRY:
            return "invalid_geometry";
    }
    return "invalid_geometry";
}

PairBridgeTtcCorrection evaluateBridgeTtcCorrection(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PredictedKinematicSample>& prediction_a,
    const std::vector<PredictedKinematicSample>& prediction_b,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config) {
    PairBridgeTtcCorrection result;
    if (!nominal_baseline.event.valid) return result;
    result.baseline_conflict = true;
    const double original_ttc_a = predictionTimeAtS(
        prediction_a, nominal_baseline.event.collision_s_a);
    const double original_ttc_b = predictionTimeAtS(
        prediction_b, nominal_baseline.event.collision_s_b);
    result.a = evaluateVehicle(
        vehicle_a, vehicle_b, prediction_a,
        nominal_baseline.event.collision_s_a,
        nominal_baseline.event.collision_s_b,
        original_ttc_a, map_param, config);
    result.b = evaluateVehicle(
        vehicle_b, vehicle_a, prediction_b,
        nominal_baseline.event.collision_s_b,
        nominal_baseline.event.collision_s_a,
        original_ttc_b, map_param, config);
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
