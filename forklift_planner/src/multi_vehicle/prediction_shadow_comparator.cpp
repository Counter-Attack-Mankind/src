#include "forklift_planner/multi_vehicle/prediction_shadow_comparator.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

double angleError(double a, double b) {
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

}  // namespace

const char* shadowMismatchKindName(ShadowMismatchKind kind) {
    switch (kind) {
        case ShadowMismatchKind::NONE: return "none";
        case ShadowMismatchKind::FUTURE_LIFECYCLE_SEGMENT:
            return "future_lifecycle_segment";
        case ShadowMismatchKind::OLD_PREDICTION_UNAVAILABLE:
            return "old_prediction_unavailable";
        case ShadowMismatchKind::TIME_GRID: return "time_grid";
    }
    return "unknown";
}

double LegacyPredictionShadowGenerator::curvatureSpeedAt(
    const PathTrack& track, double query_s) const {
    if (cfg_.lat_accel_max <= 0.0 || track.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const double length = track.length();
    const double s = std::max(0.0, std::min(query_s, length));
    constexpr double sample_ds = 0.05;
    const RoughWp pa = track.poseAtS(std::max(0.0, s - sample_ds));
    const RoughWp pb = track.poseAtS(s);
    const RoughWp pc = track.poseAtS(std::min(length, s + sample_ds));
    const double abx = pb.x - pa.x;
    const double aby = pb.y - pa.y;
    const double acx = pc.x - pa.x;
    const double acy = pc.y - pa.y;
    const double lab = std::hypot(abx, aby);
    const double lbc = std::hypot(pc.x - pb.x, pc.y - pb.y);
    const double lac = std::hypot(acx, acy);
    if (lab < 1e-4 || lbc < 1e-4 || lac < 1e-4) {
        return std::numeric_limits<double>::infinity();
    }
    const double kappa =
        2.0 * std::abs(abx * acy - aby * acx) / (lab * lbc * lac);
    if (kappa < 1e-3) return std::numeric_limits<double>::infinity();
    return std::max(std::sqrt(cfg_.lat_accel_max / kappa),
                    cfg_.nominal_speed * cfg_.creep_ratio);
}

std::vector<LegacyPredictionSample>
LegacyPredictionShadowGenerator::generate(const VehicleAgent& vehicle,
                                          double requested_horizon) const {
    std::vector<LegacyPredictionSample> out;
    // This gate intentionally mirrors resolvePairwiseConflicts().
    if (!vehicle.active() || vehicle.track.empty()) return out;

    const double horizon =
        std::max(cfg_.prediction_step, requested_horizon);
    const double prediction_step = std::max(0.02, cfg_.prediction_step);
    const int prediction_count = std::max(
        1, static_cast<int>(std::ceil(horizon / prediction_step)));
    const double footprint_margin = 0.5 * cfg_.conflict_margin;
    out.reserve(static_cast<std::size_t>(prediction_count + 1));

    double s = std::max(0.0, std::min(vehicle.path_s,
                                      vehicle.track.length()));
    double speed = std::max(0.0, vehicle.current_speed);
    auto emit = [&](double t) {
        const RoughWp pose = vehicle.track.poseAtS(s);
        out.push_back(LegacyPredictionSample{
            t, s, speed, pose, makeBody(pose, mp_, footprint_margin)});
    };
    emit(0.0);
    for (int k = 1; k <= prediction_count; ++k) {
        const double previous_t = (k - 1) * prediction_step;
        const double t = std::min(horizon, k * prediction_step);
        const double step = t - previous_t;
        if (step <= 1e-9) continue;
        if (s >= vehicle.track.length() - 1e-9) {
            s = vehicle.track.length();
            speed = 0.0;
        } else {
            const double desired = std::min(
                cfg_.nominal_speed, curvatureSpeedAt(vehicle.track, s));
            if (desired > speed) {
                speed = std::min(desired, speed + cfg_.max_accel * step);
            } else {
                speed = std::max(desired, speed - cfg_.max_decel * step);
            }
            s = std::min(vehicle.track.length(), s + speed * step);
        }
        emit(t);
    }
    return out;
}

PredictionShadowReport PredictionShadowComparator::compare(
    const VehicleAgent& vehicle,
    const std::vector<LegacyPredictionSample>& legacy,
    const FutureMissionTrajectory& future) const {
    PredictionShadowReport report;
    report.vehicle_id = vehicle.id;
    report.old_sample_count = legacy.size();
    report.new_sample_count = future.samples.size();

    double s_error_sum = 0.0;
    double position_error_sum = 0.0;
    const std::size_t common = std::min(legacy.size(), future.samples.size());

    auto recordMismatch = [&](const FutureSample& sample,
                              ShadowMismatchKind kind) {
        ++report.segment_mismatch_count;
        if (!report.first_mismatch.valid) {
            report.first_mismatch.valid = true;
            report.first_mismatch.t = sample.t;
            report.first_mismatch.new_s = sample.path_s;
            report.first_mismatch.new_phase = sample.phase;
            report.first_mismatch.new_segment_id = sample.segment_id;
            report.first_mismatch.mismatch = kind;
        }
    };

    if (legacy.empty()) {
        for (const FutureSample& sample : future.samples) {
            recordMismatch(sample,
                           ShadowMismatchKind::OLD_PREDICTION_UNAVAILABLE);
        }
        return report;
    }

    for (std::size_t i = 0; i < common; ++i) {
        const LegacyPredictionSample& old_sample = legacy[i];
        const FutureSample& new_sample = future.samples[i];
        if (std::abs(old_sample.t - new_sample.t) > 1e-9) {
            ++report.time_mismatch_count;
            recordMismatch(new_sample, ShadowMismatchKind::TIME_GRID);
            continue;
        }

        const bool current_motion_segment =
            new_sample.mission_leg_id.expected_path_gen == vehicle.path_gen &&
            new_sample.phase == vehicle.mission_phase;
        if (!current_motion_segment) {
            recordMismatch(new_sample,
                           ShadowMismatchKind::FUTURE_LIFECYCLE_SEGMENT);
            continue;
        }

        const double s_error = std::abs(old_sample.s - new_sample.path_s);
        const double position_error = std::hypot(
            old_sample.pose.x - new_sample.pose.x,
            old_sample.pose.y - new_sample.pose.y);
        const double speed_error =
            std::abs(old_sample.speed - new_sample.speed);
        const double body_center_error = std::hypot(
            old_sample.body.x - new_sample.body.x,
            old_sample.body.y - new_sample.body.y);
        const double body_yaw_error =
            angleError(old_sample.body.theta, new_sample.body.theta);

        ++report.matched_sample_count;
        s_error_sum += s_error;
        position_error_sum += position_error;
        report.max_s_error = std::max(report.max_s_error, s_error);
        report.max_position_error =
            std::max(report.max_position_error, position_error);
        report.max_speed_error =
            std::max(report.max_speed_error, speed_error);
        report.max_body_center_error =
            std::max(report.max_body_center_error, body_center_error);
        report.max_body_yaw_error =
            std::max(report.max_body_yaw_error, body_yaw_error);
        if (!report.maximum_error.valid ||
            position_error > report.maximum_error.position_error) {
            report.maximum_error.valid = true;
            report.maximum_error.t = new_sample.t;
            report.maximum_error.old_s = old_sample.s;
            report.maximum_error.new_s = new_sample.path_s;
            report.maximum_error.position_error = position_error;
            report.maximum_error.new_phase = new_sample.phase;
            report.maximum_error.new_segment_id = new_sample.segment_id;
        }
    }

    // Extra future samples cannot be paired with the legacy time grid.
    for (std::size_t i = common; i < future.samples.size(); ++i) {
        recordMismatch(future.samples[i], ShadowMismatchKind::TIME_GRID);
        ++report.time_mismatch_count;
    }
    if (legacy.size() > common) {
        report.time_mismatch_count += legacy.size() - common;
        if (!report.first_mismatch.valid) {
            report.first_mismatch.valid = true;
            report.first_mismatch.t = legacy[common].t;
            report.first_mismatch.old_s = legacy[common].s;
            report.first_mismatch.new_phase = vehicle.mission_phase;
            report.first_mismatch.new_segment_id = -1;
            report.first_mismatch.mismatch = ShadowMismatchKind::TIME_GRID;
        }
    }
    if (report.matched_sample_count > 0) {
        const double count = static_cast<double>(report.matched_sample_count);
        report.mean_s_error = s_error_sum / count;
        report.mean_position_error = position_error_sum / count;
    }
    return report;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
