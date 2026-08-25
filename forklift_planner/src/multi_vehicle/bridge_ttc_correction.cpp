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
    WpType traversal_type = WpType::FORWARD;
    std::size_t first_index = 0;
    std::size_t last_index = 0;
    std::size_t index = 0;
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
    cursor.traversal_type = track.typeAtS(seed_s);
    const auto& path = track.path();
    const auto& cumulative_s = track.cumulative_s();
    auto it = std::lower_bound(cumulative_s.begin(), cumulative_s.end(),
                               std::max(0.0, std::min(seed_s, track.length())));
    std::size_t index = it == cumulative_s.end()
        ? path.size() - 1
        : static_cast<std::size_t>(std::distance(cumulative_s.begin(), it));
    if (path[index].type != cursor.traversal_type && index > 0 &&
        path[index - 1].type == cursor.traversal_type) {
        --index;
    }
    cursor.first_index = index;
    while (cursor.first_index > 0 &&
           path[cursor.first_index - 1].type == cursor.traversal_type) {
        --cursor.first_index;
    }
    cursor.last_index = index;
    while (cursor.last_index + 1 < path.size() &&
           path[cursor.last_index + 1].type == cursor.traversal_type) {
        ++cursor.last_index;
    }
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

NearestMatch nearestOnTraversal(const RoughWp& query, NearestCursor& cursor,
                                int& evaluations) {
    const auto& path = cursor.track->path();
    double best_sq = squaredDistance(query, path[cursor.index], evaluations);
    for (;;) {
        std::size_t best_index = cursor.index;
        double candidate_sq = best_sq;
        if (cursor.index > cursor.first_index) {
            const double value = squaredDistance(
                query, path[cursor.index - 1], evaluations);
            if (value + kEpsilon < candidate_sq) {
                candidate_sq = value;
                best_index = cursor.index - 1;
            }
        }
        if (cursor.index < cursor.last_index) {
            const double value = squaredDistance(
                query, path[cursor.index + 1], evaluations);
            if (value + kEpsilon < candidate_sq) {
                candidate_sq = value;
                best_index = cursor.index + 1;
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
    if (cursor.index > cursor.first_index) {
        consider(cursor.index - 1, cursor.index);
    }
    if (cursor.index < cursor.last_index) {
        consider(cursor.index, cursor.index + 1);
    }
    return best;
}

double motionHeading(const PathTrack& track, double s) {
    const RoughWp pose = track.poseAtS(s);
    return pose.theta +
        (track.typeAtS(s) == WpType::REVERSE ? kPi : 0.0);
}

double timeAtS(const std::vector<PredictedKinematicSample>& prediction,
               double target_s) {
    if (prediction.empty()) return std::numeric_limits<double>::infinity();
    if (target_s <= prediction.front().s + kEpsilon) return 0.0;
    for (std::size_t i = 1; i < prediction.size(); ++i) {
        if (prediction[i].s + kEpsilon < target_s) continue;
        const double ds = prediction[i].s - prediction[i - 1].s;
        if (ds <= kEpsilon) return prediction[i].t;
        const double ratio = std::max(0.0, std::min(
            1.0, (target_s - prediction[i - 1].s) / ds));
        return prediction[i - 1].t +
            ratio * (prediction[i].t - prediction[i - 1].t);
    }
    return std::numeric_limits<double>::infinity();
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
    if (self.track.empty() || other.track.empty() ||
        self_prediction.empty()) {
        return result;
    }

    const double proximity_limit =
        map_param.vehicle_width + config.conflict_margin;
    const WpType self_seed_type = self.track.typeAtS(collision_s);
    NearestCursor other_cursor = makeNearestCursor(
        other.track, other_collision_s);

    auto relationAt = [&](double self_s, double& match_s,
                          double& direction_dot, double& distance) {
        const RoughWp self_pose = self.track.poseAtS(self_s);
        const NearestMatch match = nearestOnTraversal(
            self_pose, other_cursor, result.nearest_search_evaluations);
        match_s = match.s;
        distance = match.distance;
        direction_dot = std::cos(
            motionHeading(self.track, self_s) -
            motionHeading(other.track, match.s));
        return match.distance <= proximity_limit + kEpsilon &&
               direction_dot < config.bridge_opposing_threshold;
    };

    ++result.backtrack_samples;
    if (!relationAt(collision_s, result.matched_other_s,
                    result.collision_direction_dot,
                    result.collision_match_distance)) {
        return result;
    }
    result.bridge_related = true;

    const double step = std::max(0.005, config.bridge_backtrack_step);
    double cursor_s = collision_s;
    while (cursor_s > 0.0 + kEpsilon) {
        const double query_s = std::max(0.0, cursor_s - step);
        if (self.track.typeAtS(query_s) != self_seed_type) break;
        double match_s = 0.0;
        double direction_dot = 1.0;
        double distance = std::numeric_limits<double>::infinity();
        ++result.backtrack_samples;
        if (!relationAt(query_s, match_s, direction_dot, distance)) break;
        result.near_boundary_s = query_s;
        cursor_s = query_s;
        if (query_s <= kEpsilon) break;
    }

    const double boundary_ttc = timeAtS(
        self_prediction, result.near_boundary_s);
    result.corrected_ttc = std::min(original_ttc, boundary_ttc);
    return result;
}

}  // namespace

PairBridgeTtcCorrection evaluateBridgeTtcCorrection(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<PredictedKinematicSample>& prediction_a,
    const std::vector<PredictedKinematicSample>& prediction_b,
    const PairInteractionResult& nominal_baseline,
    const MapParam& map_param, const MultiVehicleConfig& config) {
    PairBridgeTtcCorrection result;
    if (!nominal_baseline.event.valid) return result;
    result.baseline_conflict = true;
    result.a = evaluateVehicle(
        vehicle_a, vehicle_b, prediction_a,
        nominal_baseline.event.first_s_a,
        nominal_baseline.event.first_s_b,
        nominal_baseline.event.first_t, map_param, config);
    result.b = evaluateVehicle(
        vehicle_b, vehicle_a, prediction_b,
        nominal_baseline.event.first_s_b,
        nominal_baseline.event.first_s_a,
        nominal_baseline.event.first_t, map_param, config);
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
