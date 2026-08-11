#include "forklift_planner/multi_vehicle/future_conflict_zone_shadow.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

struct OverlapSample {
    double s_a = 0.0;
    double s_b = 0.0;
    double x = 0.0;
    double y = 0.0;
};

void hashBytes(std::uint64_t& hash, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
    }
}

}  // namespace

const char* futureConflictZoneSourceName(FutureConflictZoneSource source) {
    switch (source) {
        case FutureConflictZoneSource::CURRENT_TRACK:
            return "CURRENT_TRACK";
        case FutureConflictZoneSource::FUTURE_SEGMENT:
            return "FUTURE_SEGMENT";
    }
    return "UNKNOWN";
}

std::uint64_t FutureConflictZoneShadowBuilder::trackSignature(
    const PathTrack& track) const {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto& path = track.path();
    const std::size_t count = path.size();
    hashBytes(hash, &count, sizeof(count));
    for (const RoughWp& waypoint : path) {
        hashBytes(hash, &waypoint.x, sizeof(waypoint.x));
        hashBytes(hash, &waypoint.y, sizeof(waypoint.y));
        hashBytes(hash, &waypoint.theta, sizeof(waypoint.theta));
        const int type = static_cast<int>(waypoint.type);
        hashBytes(hash, &type, sizeof(type));
    }
    return hash;
}

std::vector<FutureConflictZone> FutureConflictZoneShadowBuilder::compute(
    int vehicle_a, int vehicle_b, const FutureMissionSegment& segment_a,
    const FutureMissionSegment& segment_b,
    FutureConflictZoneSource source) const {
    constexpr double kStep = 0.025;
    constexpr double kMergeGap = kStep * 2.25;
    std::vector<FutureConflictZone> zones;
    if (segment_a.track.empty() || segment_b.track.empty()) return zones;

    const double end_a = segment_a.track.length();
    const double end_b = segment_b.track.length();
    {
        const double inf = std::numeric_limits<double>::infinity();
        const double inflation =
            0.5 * std::hypot(mp_.vehicle_length, mp_.vehicle_width) +
            cfg_.conflict_margin;
        constexpr double kCoarse = 0.15;
        double ax0 = inf, ay0 = inf, ax1 = -inf, ay1 = -inf;
        for (double s = 0.0; s <= end_a + 1e-9; s += kCoarse) {
            const RoughWp p = segment_a.track.poseAtS(std::min(s, end_a));
            ax0 = std::min(ax0, p.x); ay0 = std::min(ay0, p.y);
            ax1 = std::max(ax1, p.x); ay1 = std::max(ay1, p.y);
        }
        double bx0 = inf, by0 = inf, bx1 = -inf, by1 = -inf;
        for (double s = 0.0; s <= end_b + 1e-9; s += kCoarse) {
            const RoughWp p = segment_b.track.poseAtS(std::min(s, end_b));
            bx0 = std::min(bx0, p.x); by0 = std::min(by0, p.y);
            bx1 = std::max(bx1, p.x); by1 = std::max(by1, p.y);
        }
        if (ax1 + inflation < bx0 - inflation ||
            bx1 + inflation < ax0 - inflation ||
            ay1 + inflation < by0 - inflation ||
            by1 + inflation < ay0 - inflation) {
            return zones;
        }
    }

    const double margin = 0.5 * cfg_.conflict_margin;
    for (double sa = 0.0; sa <= end_a + 1e-9; sa += kStep) {
        const double a_s = std::min(sa, end_a);
        const OBB body_a = makeBody(segment_a.track.poseAtS(a_s), mp_, margin);
        std::vector<OverlapSample> row;
        for (double sb = 0.0; sb <= end_b + 1e-9; sb += kStep) {
            const double b_s = std::min(sb, end_b);
            const OBB body_b =
                makeBody(segment_b.track.poseAtS(b_s), mp_, margin);
            if (!overlaps(body_a, body_b)) continue;
            const RoughWp pa = segment_a.track.poseAtS(a_s);
            const RoughWp pb = segment_b.track.poseAtS(b_s);
            row.push_back(OverlapSample{a_s, b_s,
                                        0.5 * (pa.x + pb.x),
                                        0.5 * (pa.y + pb.y)});
        }
        if (row.empty()) continue;

        std::vector<FutureConflictZone> row_zones;
        for (const OverlapSample& sample : row) {
            if (row_zones.empty() ||
                sample.s_b > row_zones.back().s_b_exit + kMergeGap) {
                FutureConflictZone zone;
                zone.s_a_enter = sample.s_a;
                zone.s_a_exit = sample.s_a;
                zone.s_b_enter = sample.s_b;
                zone.s_b_exit = sample.s_b;
                zone.x = sample.x;
                zone.y = sample.y;
                row_zones.push_back(zone);
            } else {
                FutureConflictZone& zone = row_zones.back();
                zone.s_b_exit = sample.s_b;
                zone.x = 0.5 * (zone.x + sample.x);
                zone.y = 0.5 * (zone.y + sample.y);
            }
        }

        for (const FutureConflictZone& row_zone : row_zones) {
            bool merged = false;
            for (FutureConflictZone& zone : zones) {
                const bool a_touch =
                    row_zone.s_a_enter <= zone.s_a_exit + kMergeGap;
                const bool b_touch =
                    row_zone.s_b_enter <= zone.s_b_exit + kMergeGap &&
                    row_zone.s_b_exit + kMergeGap >= zone.s_b_enter;
                if (!a_touch || !b_touch) continue;
                zone.s_a_enter = std::min(zone.s_a_enter,
                                          row_zone.s_a_enter);
                zone.s_a_exit = std::max(zone.s_a_exit,
                                         row_zone.s_a_exit);
                zone.s_b_enter = std::min(zone.s_b_enter,
                                          row_zone.s_b_enter);
                zone.s_b_exit = std::max(zone.s_b_exit,
                                         row_zone.s_b_exit);
                zone.x = 0.5 * (zone.x + row_zone.x);
                zone.y = 0.5 * (zone.y + row_zone.y);
                merged = true;
                break;
            }
            if (!merged) zones.push_back(row_zone);
        }
    }

    for (FutureConflictZone& zone : zones) {
        zone.vehicle_a = vehicle_a;
        zone.vehicle_b = vehicle_b;
        zone.segment_id_a = segment_a.segment_id;
        zone.segment_id_b = segment_b.segment_id;
        zone.phase_a = segment_a.phase;
        zone.phase_b = segment_b.phase;
        zone.certainty_a = segment_a.certainty;
        zone.certainty_b = segment_b.certainty;
        zone.path_generation_a =
            segment_a.mission_leg_id.expected_path_gen;
        zone.path_generation_b =
            segment_b.mission_leg_id.expected_path_gen;
        zone.source = source;
    }
    return zones;
}

std::vector<FutureConflictZone> FutureConflictZoneShadowBuilder::build(
    const std::vector<FutureMissionTrajectory>& trajectories) const {
    std::vector<FutureConflictZone> result;
    int next_zone_id = 0;
    for (std::size_t i = 0; i < trajectories.size(); ++i) {
        const auto& trajectory_a = trajectories[i];
        for (std::size_t j = i + 1; j < trajectories.size(); ++j) {
            const auto& trajectory_b = trajectories[j];
            for (const FutureMissionSegment& segment_a :
                 trajectory_a.plan.segments) {
                if (segment_a.type != FutureSegmentType::MOTION ||
                    segment_a.track.empty()) {
                    continue;
                }
                for (const FutureMissionSegment& segment_b :
                     trajectory_b.plan.segments) {
                    if (segment_b.type != FutureSegmentType::MOTION ||
                        segment_b.track.empty()) {
                        continue;
                    }
                    const bool both_current =
                        segment_a.mission_leg_id.expected_path_gen ==
                            trajectory_a.plan.source_path_gen &&
                        segment_b.mission_leg_id.expected_path_gen ==
                            trajectory_b.plan.source_path_gen;
                    const FutureConflictZoneSource source = both_current
                        ? FutureConflictZoneSource::CURRENT_TRACK
                        : FutureConflictZoneSource::FUTURE_SEGMENT;
                    const CacheKey key{
                        trajectory_a.plan.vehicle_id,
                        trajectory_b.plan.vehicle_id,
                        segment_a.segment_id, segment_b.segment_id,
                        segment_a.mission_leg_id.expected_path_gen,
                        segment_b.mission_leg_id.expected_path_gen,
                        trackSignature(segment_a.track),
                        trackSignature(segment_b.track)};
                    auto it = cache_.find(key);
                    if (it == cache_.end()) {
                        it = cache_.emplace(
                            key, compute(trajectory_a.plan.vehicle_id,
                                         trajectory_b.plan.vehicle_id,
                                         segment_a, segment_b, source)).first;
                    }
                    for (FutureConflictZone zone : it->second) {
                        zone.future_zone_id = next_zone_id++;
                        // The cache owns only static geometry.  Refresh all
                        // horizon-scoped segment metadata so a reused track
                        // cannot retain an earlier plan's labels.
                        zone.vehicle_a = trajectory_a.plan.vehicle_id;
                        zone.vehicle_b = trajectory_b.plan.vehicle_id;
                        zone.segment_id_a = segment_a.segment_id;
                        zone.segment_id_b = segment_b.segment_id;
                        zone.phase_a = segment_a.phase;
                        zone.phase_b = segment_b.phase;
                        zone.certainty_a = segment_a.certainty;
                        zone.certainty_b = segment_b.certainty;
                        zone.path_generation_a =
                            segment_a.mission_leg_id.expected_path_gen;
                        zone.path_generation_b =
                            segment_b.mission_leg_id.expected_path_gen;
                        zone.source = source;
                        result.push_back(std::move(zone));
                    }
                }
            }
        }
    }
    return result;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
