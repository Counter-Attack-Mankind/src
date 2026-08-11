#include "forklift_planner/multi_vehicle/timed_conflict_shadow_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

double intervalDistance(double value, double begin, double end) {
    if (value < begin) return begin - value;
    if (value > end) return value - end;
    return 0.0;
}

int nearestZone(int vehicle_a, int vehicle_b, double s_a, double s_b,
                const std::vector<ShadowConflictZone>& zones) {
    double best_score = std::numeric_limits<double>::infinity();
    int best_zone = -1;
    for (const ShadowConflictZone& zone : zones) {
        if (zone.vehicle_a != vehicle_a || zone.vehicle_b != vehicle_b) {
            continue;
        }
        const double score =
            intervalDistance(s_a, zone.s_a_enter, zone.s_a_exit) +
            intervalDistance(s_b, zone.s_b_enter, zone.s_b_exit);
        if (score < best_score) {
            best_score = score;
            best_zone = zone.zone_index;
        }
    }
    return best_zone;
}

int nearestFutureZone(int vehicle_a, int vehicle_b, int segment_a,
                      int segment_b, double s_a, double s_b,
                      const std::vector<FutureConflictZone>& zones) {
    double best_score = std::numeric_limits<double>::infinity();
    int best_zone = -1;
    for (const FutureConflictZone& zone : zones) {
        if (zone.vehicle_a != vehicle_a || zone.vehicle_b != vehicle_b ||
            zone.segment_id_a != segment_a ||
            zone.segment_id_b != segment_b) {
            continue;
        }
        const double score =
            intervalDistance(s_a, zone.s_a_enter, zone.s_a_exit) +
            intervalDistance(s_b, zone.s_b_enter, zone.s_b_exit);
        if (score < best_score) {
            best_score = score;
            best_zone = zone.future_zone_id;
        }
    }
    return best_zone;
}

bool futureLifecycleSample(const VehicleAgent& vehicle,
                           const FutureSample& sample) {
    return sample.mission_leg_id.expected_path_gen != vehicle.path_gen ||
           sample.phase != vehicle.mission_phase;
}

}  // namespace

const char* timedConflictShadowClassName(TimedConflictShadowClass value) {
    switch (value) {
        case TimedConflictShadowClass::MATCH: return "MATCH";
        case TimedConflictShadowClass::NEW_FUTURE_LIFECYCLE_CONFLICT:
            return "NEW_FUTURE_LIFECYCLE_CONFLICT";
        case TimedConflictShadowClass::PREDICTION_ERROR:
            return "PREDICTION_ERROR";
        case TimedConflictShadowClass::ZONE_MAPPING_DIFFERENCE:
            return "ZONE_MAPPING_DIFFERENCE";
    }
    return "UNKNOWN";
}

ShadowTimedEvent TimedConflictShadowChecker::checkLegacy(
    int vehicle_a, int vehicle_b,
    const std::vector<LegacyPredictionSample>& samples_a,
    const std::vector<LegacyPredictionSample>& samples_b,
    const std::vector<ShadowConflictZone>& zones) const {
    ShadowTimedEvent event;
    event.vehicle_a = vehicle_a;
    event.vehicle_b = vehicle_b;
    const std::size_t count = std::min(samples_a.size(), samples_b.size());
    for (std::size_t k = 0; k < count; ++k) {
        if (std::abs(samples_a[k].t - samples_b[k].t) > 1e-9) continue;
        const bool hit = overlaps(samples_a[k].body, samples_b[k].body);
        if (!hit) {
            if (event.valid) break;
            continue;
        }
        if (!event.valid) {
            event.valid = true;
            event.first_t = samples_a[k].t;
            event.matched_zone = nearestZone(
                vehicle_a, vehicle_b, samples_a[k].s, samples_b[k].s,
                zones);
            event.x = 0.5 * (samples_a[k].body.x + samples_b[k].body.x);
            event.y = 0.5 * (samples_a[k].body.y + samples_b[k].body.y);
        }
        event.last_t = samples_a[k].t;
        ++event.overlap_samples;
    }
    if (event.valid) event.overlap_duration = event.last_t - event.first_t;
    return event;
}

ShadowTimedEvent TimedConflictShadowChecker::checkFuture(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const FutureMissionTrajectory& trajectory_a,
    const FutureMissionTrajectory& trajectory_b,
    const std::vector<ShadowConflictZone>& zones,
    const std::vector<FutureConflictZone>& future_zones) const {
    ShadowTimedEvent event;
    event.vehicle_a = vehicle_a.id;
    event.vehicle_b = vehicle_b.id;
    const std::size_t count =
        std::min(trajectory_a.samples.size(), trajectory_b.samples.size());
    const double margin = 0.5 * cfg_.conflict_margin;
    for (std::size_t k = 0; k < count; ++k) {
        const FutureSample& sample_a = trajectory_a.samples[k];
        const FutureSample& sample_b = trajectory_b.samples[k];
        if (std::abs(sample_a.t - sample_b.t) > 1e-9) continue;
        const OBB body_a = makeBody(sample_a.pose, mp_, margin);
        const OBB body_b = makeBody(sample_b.pose, mp_, margin);
        const bool hit = overlaps(body_a, body_b);
        if (!hit) {
            if (event.valid) break;
            continue;
        }
        if (!event.valid) {
            event.valid = true;
            event.first_t = sample_a.t;
            event.segment_a = sample_a.segment_id;
            event.segment_b = sample_b.segment_id;
            event.certainty_a = sample_a.certainty;
            event.certainty_b = sample_b.certainty;
            event.phase_a = sample_a.phase;
            event.phase_b = sample_b.phase;
            event.future_zone_id = nearestFutureZone(
                vehicle_a.id, vehicle_b.id, sample_a.segment_id,
                sample_b.segment_id, sample_a.path_s, sample_b.path_s,
                future_zones);
            // Existing ConflictZones describe the two current tracks only.
            // Never attach a cross-lifecycle sample to that geometry.
            if (!futureLifecycleSample(vehicle_a, sample_a) &&
                !futureLifecycleSample(vehicle_b, sample_b)) {
                event.matched_zone = nearestZone(
                    vehicle_a.id, vehicle_b.id, sample_a.path_s,
                    sample_b.path_s, zones);
            }
            event.x = 0.5 * (body_a.x + body_b.x);
            event.y = 0.5 * (body_a.y + body_b.y);
        }
        event.last_t = sample_a.t;
        ++event.overlap_samples;
    }
    if (event.valid) event.overlap_duration = event.last_t - event.first_t;
    return event;
}

TimedConflictShadowReport TimedConflictShadowChecker::compare(
    const VehicleAgent& vehicle_a, const VehicleAgent& vehicle_b,
    const std::vector<LegacyPredictionSample>& legacy_a,
    const std::vector<LegacyPredictionSample>& legacy_b,
    const FutureMissionTrajectory& future_a,
    const FutureMissionTrajectory& future_b,
    const std::vector<ShadowConflictZone>& zones,
    const std::vector<FutureConflictZone>& future_zones) const {
    TimedConflictShadowReport report;
    report.vehicle_a = vehicle_a.id;
    report.vehicle_b = vehicle_b.id;
    report.old_event = checkLegacy(vehicle_a.id, vehicle_b.id, legacy_a,
                                   legacy_b, zones);
    report.new_event = checkFuture(vehicle_a, vehicle_b, future_a, future_b,
                                   zones, future_zones);

    const bool new_lifecycle = report.new_event.valid &&
        (legacy_a.empty() || legacy_b.empty() ||
         report.new_event.segment_a < 0 || report.new_event.segment_b < 0 ||
         report.new_event.phase_a != vehicle_a.mission_phase ||
         report.new_event.phase_b != vehicle_b.mission_phase ||
         report.new_event.certainty_a != FutureCertainty::COMMITTED ||
         report.new_event.certainty_b != FutureCertainty::COMMITTED);
    if (!report.old_event.valid && report.new_event.valid && new_lifecycle) {
        report.classification =
            TimedConflictShadowClass::NEW_FUTURE_LIFECYCLE_CONFLICT;
        return report;
    }
    if (report.old_event.valid != report.new_event.valid) {
        report.classification = TimedConflictShadowClass::PREDICTION_ERROR;
        return report;
    }
    if (!report.old_event.valid) return report;

    const double tolerance = std::max(0.02, cfg_.prediction_step) + 1e-9;
    if (std::abs(report.old_event.first_t - report.new_event.first_t) >
            tolerance ||
        std::abs(report.old_event.last_t - report.new_event.last_t) >
            tolerance) {
        report.classification = new_lifecycle
            ? TimedConflictShadowClass::NEW_FUTURE_LIFECYCLE_CONFLICT
            : TimedConflictShadowClass::PREDICTION_ERROR;
    } else if (report.old_event.matched_zone !=
               report.new_event.matched_zone) {
        report.classification = new_lifecycle
            ? TimedConflictShadowClass::NEW_FUTURE_LIFECYCLE_CONFLICT
            : TimedConflictShadowClass::ZONE_MAPPING_DIFFERENCE;
    }
    return report;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
