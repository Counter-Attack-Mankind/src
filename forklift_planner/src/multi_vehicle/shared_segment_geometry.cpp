#include "forklift_planner/multi_vehicle/shared_segment_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"

namespace forklift_planner {
namespace multi_vehicle {
namespace {

constexpr double kScanStep = 0.025;
constexpr double kOpposingDirectionDot = -0.5;
constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

struct TraversalSpan {
    int id = -1;
    WpType direction = WpType::FORWARD;
    double s_enter = 0.0;
    double s_exit = 0.0;
};

struct TraversalTable {
    const PathTrack* track = nullptr;
    std::vector<TraversalSpan> spans;

    const TraversalSpan& at(double s) const {
        const double clamped = std::max(0.0, std::min(s, track->length()));
        const WpType direction = track->typeAtS(clamped);
        const TraversalSpan* nearest = nullptr;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (const TraversalSpan& span : spans) {
            if (span.direction != direction) continue;
            if (clamped + kEpsilon >= span.s_enter &&
                clamped <= span.s_exit + kEpsilon) {
                return span;
            }
            const double distance = clamped < span.s_enter
                ? span.s_enter - clamped : clamped - span.s_exit;
            if (distance < nearest_distance) {
                nearest = &span;
                nearest_distance = distance;
            }
        }
        // A non-empty PathTrack always produces a matching span. Keep a
        // deterministic fallback for malformed/zero-length waypoint runs.
        return nearest != nullptr ? *nearest : spans.front();
    }
};

struct OverlapSample {
    int grid_a = 0;
    int grid_b = 0;
    double s_a = 0.0;
    double s_b = 0.0;
    int traversal_a = -1;
    int traversal_b = -1;
    WpType direction_a = WpType::FORWARD;
    WpType direction_b = WpType::FORWARD;
    double direction_dot = 0.0;
    double aabb_min_x = 0.0;
    double aabb_min_y = 0.0;
    double aabb_max_x = 0.0;
    double aabb_max_y = 0.0;
    bool aabb_valid = false;
};

using TraversalPair = std::pair<int, int>;
using GridCell = std::pair<int, int>;

TraversalTable buildTraversalTable(const PathTrack& track) {
    TraversalTable table;
    table.track = &track;
    if (track.empty()) return table;

    const RoughPath& path = track.path();
    const std::vector<double>& cumulative_s = track.cumulative_s();
    std::vector<std::size_t> run_starts{0};
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (path[i].type != path[i - 1].type) run_starts.push_back(i);
    }

    table.spans.reserve(run_starts.size());
    double previous_boundary = 0.0;
    for (std::size_t run = 0; run < run_starts.size(); ++run) {
        const std::size_t start = run_starts[run];
        double next_boundary = track.length();
        if (run + 1 < run_starts.size()) {
            const std::size_t next = run_starts[run + 1];
            const double before_s = cumulative_s[next - 1];
            const double after_s = cumulative_s[next];
            next_boundary = 0.5 * (before_s + after_s);
        }
        table.spans.push_back(TraversalSpan{
            static_cast<int>(run), path[start].type,
            previous_boundary, next_boundary});
        previous_boundary = next_boundary;
    }
    return table;
}

double motionHeading(const RoughWp& pose, WpType direction) {
    return pose.theta + (direction == WpType::REVERSE ? kPi : 0.0);
}

void setIntersectionAabb(const OBB& a, const OBB& b, OverlapSample& sample) {
    const std::vector<InteractionPoint> polygon = intersectObbs(a, b);
    if (polygon.size() < 3) return;
    sample.aabb_min_x = sample.aabb_max_x = polygon.front().x;
    sample.aabb_min_y = sample.aabb_max_y = polygon.front().y;
    for (const InteractionPoint& point : polygon) {
        sample.aabb_min_x = std::min(sample.aabb_min_x, point.x);
        sample.aabb_min_y = std::min(sample.aabb_min_y, point.y);
        sample.aabb_max_x = std::max(sample.aabb_max_x, point.x);
        sample.aabb_max_y = std::max(sample.aabb_max_y, point.y);
    }
    sample.aabb_valid = true;
}

void mergeSample(SharedSegmentCandidate& candidate,
                 const OverlapSample& sample,
                 double strong_opposing_threshold,
                 double& direction_dot_sum) {
    candidate.s_a_enter = std::min(candidate.s_a_enter, sample.s_a);
    candidate.s_a_exit = std::max(candidate.s_a_exit, sample.s_a);
    candidate.s_b_enter = std::min(candidate.s_b_enter, sample.s_b);
    candidate.s_b_exit = std::max(candidate.s_b_exit, sample.s_b);
    candidate.direction_dot_min =
        std::min(candidate.direction_dot_min, sample.direction_dot);
    candidate.direction_dot_max =
        std::max(candidate.direction_dot_max, sample.direction_dot);
    direction_dot_sum += sample.direction_dot;
    ++candidate.sample_count;
    if (sample.direction_dot < strong_opposing_threshold) {
        ++candidate.strong_opposing_count;
    }
    if (!sample.aabb_valid) return;
    if (!candidate.aabb_valid) {
        candidate.aabb_min_x = sample.aabb_min_x;
        candidate.aabb_min_y = sample.aabb_min_y;
        candidate.aabb_max_x = sample.aabb_max_x;
        candidate.aabb_max_y = sample.aabb_max_y;
        candidate.aabb_valid = true;
        return;
    }
    candidate.aabb_min_x = std::min(candidate.aabb_min_x, sample.aabb_min_x);
    candidate.aabb_min_y = std::min(candidate.aabb_min_y, sample.aabb_min_y);
    candidate.aabb_max_x = std::max(candidate.aabb_max_x, sample.aabb_max_x);
    candidate.aabb_max_y = std::max(candidate.aabb_max_y, sample.aabb_max_y);
}

}  // namespace

std::vector<SharedSegmentCandidate> computeSharedSegmentCandidates(
    const PathTrack& track_a, int path_gen_a,
    const PathTrack& track_b, int path_gen_b,
    const MapParam& map_param, const MultiVehicleConfig& config) {
    std::vector<SharedSegmentCandidate> candidates;
    if (track_a.empty() || track_b.empty()) return candidates;

    const TraversalTable traversals_a = buildTraversalTable(track_a);
    const TraversalTable traversals_b = buildTraversalTable(track_b);
    const double footprint_margin = 0.5 * config.conflict_margin;
    std::map<TraversalPair, std::vector<OverlapSample>> groups;

    for (int grid_a = 0;; ++grid_a) {
        const double raw_s_a = static_cast<double>(grid_a) * kScanStep;
        if (raw_s_a > track_a.length() + kEpsilon) break;
        const double s_a = std::min(raw_s_a, track_a.length());
        const RoughWp pose_a = track_a.poseAtS(s_a);
        const TraversalSpan& traversal_a = traversals_a.at(s_a);
        const OBB body_a = makeBody(pose_a, map_param, footprint_margin);

        for (int grid_b = 0;; ++grid_b) {
            const double raw_s_b = static_cast<double>(grid_b) * kScanStep;
            if (raw_s_b > track_b.length() + kEpsilon) break;
            const double s_b = std::min(raw_s_b, track_b.length());
            const RoughWp pose_b = track_b.poseAtS(s_b);
            const OBB body_b = makeBody(pose_b, map_param, footprint_margin);
            if (!overlaps(body_a, body_b)) continue;

            const TraversalSpan& traversal_b = traversals_b.at(s_b);
            const double direction_dot = std::cos(
                motionHeading(pose_a, traversal_a.direction) -
                motionHeading(pose_b, traversal_b.direction));
            if (direction_dot >= kOpposingDirectionDot) continue;

            OverlapSample sample;
            sample.grid_a = grid_a;
            sample.grid_b = grid_b;
            sample.s_a = s_a;
            sample.s_b = s_b;
            sample.traversal_a = traversal_a.id;
            sample.traversal_b = traversal_b.id;
            sample.direction_a = traversal_a.direction;
            sample.direction_b = traversal_b.direction;
            sample.direction_dot = direction_dot;
            setIntersectionAabb(body_a, body_b, sample);
            groups[{sample.traversal_a, sample.traversal_b}].push_back(sample);
        }
    }

    for (const auto& group_entry : groups) {
        const std::vector<OverlapSample>& samples = group_entry.second;
        std::map<GridCell, std::size_t> sample_at;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            sample_at[{samples[i].grid_a, samples[i].grid_b}] = i;
        }
        std::vector<bool> visited(samples.size(), false);
        for (std::size_t seed = 0; seed < samples.size(); ++seed) {
            if (visited[seed]) continue;
            visited[seed] = true;
            std::queue<std::size_t> open;
            open.push(seed);

            const OverlapSample& first = samples[seed];
            SharedSegmentCandidate candidate;
            candidate.path_gen_a = path_gen_a;
            candidate.path_gen_b = path_gen_b;
            candidate.traversal_a = first.traversal_a;
            candidate.traversal_b = first.traversal_b;
            candidate.direction_a = first.direction_a;
            candidate.direction_b = first.direction_b;
            candidate.s_a_enter = candidate.s_a_exit = first.s_a;
            candidate.s_b_enter = candidate.s_b_exit = first.s_b;
            candidate.direction_dot_min = first.direction_dot;
            candidate.direction_dot_max = first.direction_dot;
            candidate.sample_count = 0;
            candidate.strong_opposing_count = 0;
            double direction_dot_sum = 0.0;

            while (!open.empty()) {
                const std::size_t current_index = open.front();
                open.pop();
                const OverlapSample& current = samples[current_index];
                mergeSample(candidate, current,
                            config.shared_segment_strong_opposing_threshold,
                            direction_dot_sum);
                for (int da = -1; da <= 1; ++da) {
                    for (int db = -1; db <= 1; ++db) {
                        if (da == 0 && db == 0) continue;
                        const auto found = sample_at.find(
                            {current.grid_a + da, current.grid_b + db});
                        if (found == sample_at.end() || visited[found->second]) {
                            continue;
                        }
                        visited[found->second] = true;
                        open.push(found->second);
                    }
                }
            }
            candidate.direction_dot_mean =
                direction_dot_sum / static_cast<double>(candidate.sample_count);
            candidate.strong_opposing_ratio =
                static_cast<double>(candidate.strong_opposing_count) /
                static_cast<double>(candidate.sample_count);

            // Qualification is deliberately component-level and happens only
            // after closure. Local opposing samples are never removed during
            // BFS, so a genuine shared segment cannot be fragmented by the
            // strong-direction or span thresholds.
            const double span_a = candidate.s_a_exit - candidate.s_a_enter;
            const double span_b = candidate.s_b_exit - candidate.s_b_enter;
            const bool qualified =
                span_a + kEpsilon >= config.shared_segment_min_span &&
                span_b + kEpsilon >= config.shared_segment_min_span &&
                candidate.strong_opposing_ratio + kEpsilon >=
                    config.shared_segment_min_strong_ratio;
            if (qualified) candidates.push_back(candidate);
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const SharedSegmentCandidate& lhs,
                 const SharedSegmentCandidate& rhs) {
        return std::tie(lhs.traversal_a, lhs.traversal_b,
                        lhs.s_a_enter, lhs.s_b_enter,
                        lhs.s_a_exit, lhs.s_b_exit) <
               std::tie(rhs.traversal_a, rhs.traversal_b,
                        rhs.s_a_enter, rhs.s_b_enter,
                        rhs.s_a_exit, rhs.s_b_exit);
    });
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        candidates[i].id = static_cast<int>(i);
    }
    return candidates;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
