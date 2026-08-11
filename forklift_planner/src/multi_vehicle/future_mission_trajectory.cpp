#include "forklift_planner/multi_vehicle/future_mission_trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

RoughWp vehiclePose(const VehicleAgent& vehicle) {
    if (vehicle.track.empty()) return {};
    const double s = vehicle.mode == VehicleMode::DWELL
        ? vehicle.track.length()
        : std::max(0.0, std::min(vehicle.path_s, vehicle.track.length()));
    return vehicle.track.poseAtS(s);
}

}  // namespace

const char* futureSegmentTypeName(FutureSegmentType type) {
    switch (type) {
        case FutureSegmentType::MOTION: return "MOTION";
        case FutureSegmentType::DWELL: return "DWELL";
        case FutureSegmentType::UNKNOWN: return "UNKNOWN";
    }
    return "UNKNOWN";
}

const char* futureCertaintyName(FutureCertainty certainty) {
    switch (certainty) {
        case FutureCertainty::COMMITTED: return "COMMITTED";
        case FutureCertainty::PREVIEW: return "PREVIEW";
        case FutureCertainty::UNKNOWN: return "UNKNOWN";
    }
    return "UNKNOWN";
}

FutureMissionPlan FutureMissionPlanBuilder::build(
    const VehicleAgent& vehicle, double horizon,
    const std::optional<PathTrack>& next_pickup_track) const {
    FutureMissionPlan plan;
    plan.vehicle_id = vehicle.id;
    plan.source_path_gen = vehicle.path_gen;
    plan.horizon = std::max(0.0, horizon);
    plan.initial_speed = std::max(0.0, vehicle.current_speed);

    int ordinal = 0;
    auto addSegment = [&](MissionPhase phase, FutureSegmentType type,
                          FutureCertainty certainty, const PathTrack& track,
                          double start_s, double end_s, double duration,
                          const RoughWp& hold_pose, int expected_path_gen) {
        FutureMissionSegment segment;
        segment.segment_id = ordinal;
        segment.mission_leg_id =
            MissionLegId{vehicle.id, expected_path_gen, ordinal};
        segment.phase = phase;
        segment.type = type;
        segment.certainty = certainty;
        segment.duration = duration;
        segment.track = track;
        segment.start_s = start_s;
        segment.end_s = end_s;
        segment.hold_pose = hold_pose;
        plan.segments.push_back(std::move(segment));
        ++ordinal;
    };

    const RoughWp current_pose = vehiclePose(vehicle);
    const PathTrack empty_track;
    const auto addUnknown = [&](MissionPhase phase, const RoughWp& pose,
                                int expected_path_gen) {
        addSegment(phase, FutureSegmentType::UNKNOWN,
                   FutureCertainty::UNKNOWN, empty_track, 0.0, 0.0,
                   plan.horizon, pose, expected_path_gen);
    };

    switch (vehicle.mission_phase) {
        case MissionPhase::TO_A1: {
            if (!vehicle.track.empty()) {
                addSegment(MissionPhase::TO_A1, FutureSegmentType::MOTION,
                           FutureCertainty::COMMITTED, vehicle.track,
                           vehicle.path_s, vehicle.track.length(), -1.0,
                           vehicle.track.poseAtS(vehicle.track.length()),
                           vehicle.path_gen);
            }
            const RoughWp a1_pose = vehicle.track.empty()
                ? current_pose : vehicle.track.poseAtS(vehicle.track.length());
            addSegment(MissionPhase::PICKUP_DWELL,
                       FutureSegmentType::DWELL,
                       FutureCertainty::COMMITTED, empty_track, 0.0, 0.0,
                       cfg_.pickup_dwell_time, a1_pose, vehicle.path_gen);
            if (vehicle.pending_dropoff_valid &&
                !vehicle.pending_dropoff_track.empty()) {
                addSegment(MissionPhase::TO_B, FutureSegmentType::MOTION,
                           FutureCertainty::COMMITTED,
                           vehicle.pending_dropoff_track, 0.0,
                           vehicle.pending_dropoff_track.length(), -1.0,
                           vehicle.pending_dropoff_track.poseAtS(0.0),
                           vehicle.path_gen + 1);
            } else {
                addUnknown(MissionPhase::WAIT_DROPOFF_TASK, a1_pose,
                           vehicle.path_gen);
            }
            break;
        }
        case MissionPhase::PICKUP_DWELL: {
            addSegment(MissionPhase::PICKUP_DWELL,
                       FutureSegmentType::DWELL,
                       FutureCertainty::COMMITTED, empty_track, 0.0, 0.0,
                       std::max(0.0, vehicle.dwell_remaining), current_pose,
                       vehicle.path_gen);
            if (vehicle.pending_dropoff_valid &&
                !vehicle.pending_dropoff_track.empty()) {
                addSegment(MissionPhase::TO_B, FutureSegmentType::MOTION,
                           FutureCertainty::COMMITTED,
                           vehicle.pending_dropoff_track, 0.0,
                           vehicle.pending_dropoff_track.length(), -1.0,
                           vehicle.pending_dropoff_track.poseAtS(0.0),
                           vehicle.path_gen + 1);
            } else {
                addUnknown(MissionPhase::WAIT_DROPOFF_TASK, current_pose,
                           vehicle.path_gen);
            }
            break;
        }
        case MissionPhase::TO_B: {
            if (!vehicle.track.empty()) {
                addSegment(MissionPhase::TO_B, FutureSegmentType::MOTION,
                           FutureCertainty::COMMITTED, vehicle.track,
                           vehicle.path_s, vehicle.track.length(), -1.0,
                           vehicle.track.poseAtS(vehicle.track.length()),
                           vehicle.path_gen);
            }
            const RoughWp b_pose = vehicle.track.empty()
                ? current_pose : vehicle.track.poseAtS(vehicle.track.length());
            addSegment(MissionPhase::UNLOAD_DWELL,
                       FutureSegmentType::DWELL,
                       FutureCertainty::COMMITTED, empty_track, 0.0, 0.0,
                       cfg_.unload_dwell_time, b_pose, vehicle.path_gen);
            if (next_pickup_track && !next_pickup_track->empty()) {
                addSegment(MissionPhase::TO_A1, FutureSegmentType::MOTION,
                           FutureCertainty::PREVIEW, *next_pickup_track, 0.0,
                           next_pickup_track->length(), -1.0,
                           next_pickup_track->poseAtS(0.0),
                           vehicle.path_gen + 1);
            } else {
                addUnknown(MissionPhase::TO_A1, b_pose, vehicle.path_gen + 1);
            }
            break;
        }
        case MissionPhase::UNLOAD_DWELL: {
            addSegment(MissionPhase::UNLOAD_DWELL,
                       FutureSegmentType::DWELL,
                       FutureCertainty::COMMITTED, empty_track, 0.0, 0.0,
                       std::max(0.0, vehicle.dwell_remaining), current_pose,
                       vehicle.path_gen);
            if (next_pickup_track && !next_pickup_track->empty()) {
                addSegment(MissionPhase::TO_A1, FutureSegmentType::MOTION,
                           FutureCertainty::PREVIEW, *next_pickup_track, 0.0,
                           next_pickup_track->length(), -1.0,
                           next_pickup_track->poseAtS(0.0),
                           vehicle.path_gen + 1);
            } else {
                addUnknown(MissionPhase::TO_A1, current_pose,
                           vehicle.path_gen + 1);
            }
            break;
        }
        case MissionPhase::WAIT_DROPOFF_TASK:
            addUnknown(MissionPhase::WAIT_DROPOFF_TASK, current_pose,
                       vehicle.path_gen);
            break;
        case MissionPhase::DIRECT_TO_B:
            if (vehicle.active() && !vehicle.track.empty()) {
                addSegment(MissionPhase::DIRECT_TO_B,
                           FutureSegmentType::MOTION,
                           FutureCertainty::COMMITTED, vehicle.track,
                           vehicle.path_s, vehicle.track.length(), -1.0,
                           vehicle.track.poseAtS(vehicle.track.length()),
                           vehicle.path_gen);
            } else {
                addUnknown(MissionPhase::DIRECT_TO_B, current_pose,
                           vehicle.path_gen);
            }
            break;
    }
    return plan;
}

double FutureTrajectoryGenerator::limitedSpeed(double current, double desired,
                                               double dt) const {
    if (desired > current) {
        return std::min(desired, current + cfg_.max_accel * dt);
    }
    return std::max(desired, current - cfg_.max_decel * dt);
}

double FutureTrajectoryGenerator::curvatureSpeedAt(
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

std::vector<FutureSample> FutureTrajectoryGenerator::generate(
    FutureMissionPlan& plan) const {
    std::vector<FutureSample> samples;
    if (plan.horizon < 0.0 || plan.segments.empty()) return samples;

    const double dt = std::max(0.02, cfg_.prediction_step);
    double t = 0.0;
    double speed = std::max(0.0, plan.initial_speed);

    auto emit = [&](const FutureMissionSegment& segment, double sample_t,
                    const RoughWp& pose, double sample_speed, double path_s) {
        FutureSample sample;
        sample.t = sample_t;
        sample.pose = pose;
        sample.speed = sample_speed;
        sample.body = makeBody(pose, mp_, 0.0);
        sample.phase = segment.phase;
        sample.segment_id = segment.segment_id;
        sample.mission_leg_id = segment.mission_leg_id;
        sample.path_s = path_s;
        sample.certainty = segment.certainty;
        if (!samples.empty() &&
            std::abs(samples.back().t - sample_t) <= 1e-9) {
            samples.back() = sample;
        } else {
            samples.push_back(sample);
        }
    };

    for (FutureMissionSegment& segment : plan.segments) {
        if (t > plan.horizon + 1e-9) break;
        segment.start_time = t;

        if (segment.type == FutureSegmentType::MOTION &&
            !segment.track.empty()) {
            double s = std::max(0.0, std::min(segment.start_s,
                                              segment.track.length()));
            emit(segment, t, segment.track.poseAtS(s), speed, s);
            while (t < plan.horizon - 1e-9 &&
                   s < segment.end_s - 1e-9) {
                const double step = std::min(dt, plan.horizon - t);
                const double desired = std::min(
                    cfg_.nominal_speed, curvatureSpeedAt(segment.track, s));
                speed = limitedSpeed(speed, desired, step);
                s = std::min(segment.end_s, s + speed * step);
                t += step;
                emit(segment, t, segment.track.poseAtS(s), speed, s);
                if (step <= 1e-9 || speed <= 1e-12) break;
            }
            segment.duration = t - segment.start_time;
            const bool reached_segment_end = s >= segment.end_s - 1e-9;
            if (reached_segment_end) speed = 0.0;
            // Do not fabricate the next lifecycle segment when the current
            // motion has merely been truncated by the rolling horizon.
            if (!reached_segment_end || t >= plan.horizon - 1e-9) break;
            continue;
        }

        speed = 0.0;
        const double requested_duration = segment.type == FutureSegmentType::UNKNOWN
            ? std::max(0.0, plan.horizon - t)
            : std::max(0.0, segment.duration);
        const double end_t = std::min(plan.horizon, t + requested_duration);
        emit(segment, t, segment.hold_pose, 0.0, segment.start_s);
        while (t < end_t - 1e-9) {
            t = std::min(end_t, t + dt);
            emit(segment, t, segment.hold_pose, 0.0, segment.start_s);
        }
        segment.duration = t - segment.start_time;
        if (segment.type == FutureSegmentType::UNKNOWN) break;
    }
    plan.segments.erase(
        std::remove_if(plan.segments.begin(), plan.segments.end(),
                       [](const FutureMissionSegment& segment) {
                           return segment.start_time < 0.0 ||
                                  segment.duration <= 1e-9;
                       }),
        plan.segments.end());
    return samples;
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
