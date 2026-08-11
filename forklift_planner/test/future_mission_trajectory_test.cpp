#include "forklift_planner/multi_vehicle/future_mission_trajectory.h"

#include <cmath>
#include <iostream>
#include <optional>

using forklift_planner::multi_vehicle::FutureCertainty;
using forklift_planner::multi_vehicle::FutureMissionPlanBuilder;
using forklift_planner::multi_vehicle::FutureSegmentType;
using forklift_planner::multi_vehicle::FutureTrajectoryGenerator;
using forklift_planner::multi_vehicle::LegTargetKind;
using forklift_planner::multi_vehicle::MissionPhase;
using forklift_planner::multi_vehicle::MultiVehicleConfig;
using forklift_planner::multi_vehicle::PathTrack;
using forklift_planner::multi_vehicle::VehicleAgent;
using forklift_planner::multi_vehicle::VehicleMode;

namespace {

int fail(const char* message) {
    std::cerr << "future_mission_trajectory_test: " << message << '\n';
    return 1;
}

PathTrack straight(double x0, double x1, double y = 0.0) {
    RoughPath path;
    constexpr int steps = 20;
    for (int i = 0; i <= steps; ++i) {
        const double ratio = static_cast<double>(i) / steps;
        path.push_back(RoughWp{x0 + (x1 - x0) * ratio, y, 0.0,
                               WpType::FORWARD});
    }
    PathTrack track;
    track.set(path);
    return track;
}

bool hasPhase(const std::vector<forklift_planner::multi_vehicle::FutureSample>&
                  samples,
              MissionPhase phase, FutureCertainty certainty) {
    for (const auto& sample : samples) {
        if (sample.phase == phase && sample.certainty == certainty) return true;
    }
    return false;
}

}  // namespace

int main() {
    MapParam mp;
    MultiVehicleConfig cfg;
    cfg.nominal_speed = 0.20;
    cfg.max_accel = 0.20;
    cfg.max_decel = 0.30;
    cfg.lat_accel_max = 0.10;
    cfg.prediction_step = 0.05;
    cfg.pickup_dwell_time = 5.0;
    cfg.unload_dwell_time = 5.0;

    FutureMissionPlanBuilder builder(cfg);
    FutureTrajectoryGenerator generator(mp, cfg);

    // A. TO_A1 with a frozen pending exit expands to three committed phases.
    VehicleAgent to_a1;
    to_a1.id = 0;
    to_a1.mode = VehicleMode::ACTIVE;
    to_a1.mission_phase = MissionPhase::TO_A1;
    to_a1.leg_target = LegTargetKind::A1;
    to_a1.path_gen = 7;
    to_a1.track = straight(0.0, 0.8);
    to_a1.pending_dropoff_track = straight(0.8, 1.6);
    to_a1.pending_dropoff_valid = true;
    const VehicleAgent before = to_a1;
    auto a1_plan = builder.build(to_a1, 15.0);
    if (a1_plan.segments.size() != 3 ||
        a1_plan.segments[0].phase != MissionPhase::TO_A1 ||
        a1_plan.segments[1].phase != MissionPhase::PICKUP_DWELL ||
        a1_plan.segments[2].phase != MissionPhase::TO_B) {
        return fail("TO_A1 lifecycle did not contain three expected segments");
    }
    for (const auto& segment : a1_plan.segments) {
        if (segment.certainty != FutureCertainty::COMMITTED) {
            return fail("TO_A1 lifecycle contained a non-committed segment");
        }
    }
    const auto a1_samples = generator.generate(a1_plan);
    if (!hasPhase(a1_samples, MissionPhase::TO_A1,
                  FutureCertainty::COMMITTED) ||
        !hasPhase(a1_samples, MissionPhase::PICKUP_DWELL,
                  FutureCertainty::COMMITTED) ||
        !hasPhase(a1_samples, MissionPhase::TO_B,
                  FutureCertainty::COMMITTED)) {
        return fail("TO_A1 generated samples missed a lifecycle phase");
    }
    if (to_a1.path_gen != before.path_gen ||
        to_a1.path_s != before.path_s ||
        to_a1.mission_phase != before.mission_phase ||
        to_a1.pending_dropoff_valid != before.pending_dropoff_valid) {
        return fail("shadow generation mutated VehicleAgent");
    }

    // B. TO_B expands through unload dwell into a PREVIEW B->A1 motion.
    VehicleAgent to_b;
    to_b.id = 1;
    to_b.mode = VehicleMode::ACTIVE;
    to_b.mission_phase = MissionPhase::TO_B;
    to_b.path_gen = 11;
    to_b.track = straight(0.0, 0.8, 1.0);
    const PathTrack pickup_preview = straight(0.8, 1.6, 1.0);
    auto to_b_plan = builder.build(to_b, 15.0, pickup_preview);
    if (to_b_plan.segments.size() != 3 ||
        to_b_plan.segments[0].phase != MissionPhase::TO_B ||
        to_b_plan.segments[1].phase != MissionPhase::UNLOAD_DWELL ||
        to_b_plan.segments[2].phase != MissionPhase::TO_A1 ||
        to_b_plan.segments[2].certainty != FutureCertainty::PREVIEW) {
        return fail("TO_B lifecycle did not end in TO_A1 PREVIEW");
    }
    const auto to_b_samples = generator.generate(to_b_plan);
    if (!hasPhase(to_b_samples, MissionPhase::TO_A1,
                  FutureCertainty::PREVIEW)) {
        return fail("TO_B samples missed TO_A1 PREVIEW");
    }

    // C. WAIT_DROPOFF_TASK remains an unknown stationary hold.
    VehicleAgent wait = to_a1;
    wait.id = 2;
    wait.mode = VehicleMode::DWELL;
    wait.mission_phase = MissionPhase::WAIT_DROPOFF_TASK;
    wait.pending_dropoff_valid = false;
    wait.pending_dropoff_track = PathTrack{};
    auto wait_plan = builder.build(wait, 15.0);
    if (wait_plan.segments.size() != 1 ||
        wait_plan.segments[0].type != FutureSegmentType::UNKNOWN ||
        wait_plan.segments[0].certainty != FutureCertainty::UNKNOWN) {
        return fail("WAIT_DROPOFF_TASK guessed a future task");
    }
    const auto wait_samples = generator.generate(wait_plan);
    if (wait_samples.empty() ||
        std::abs(wait_samples.back().t - 15.0) > 1e-9) {
        return fail("UNKNOWN hold did not cover the horizon");
    }
    for (const auto& sample : wait_samples) {
        if (sample.speed != 0.0) return fail("UNKNOWN hold moved");
    }

    std::cout << "A TO_A1->PICKUP_DWELL->TO_B committed=PASS\n"
              << "B TO_B->UNLOAD_DWELL->TO_A1 preview=PASS\n"
              << "C WAIT_DROPOFF_TASK unknown_hold=PASS\n"
              << "vehicle_state_unchanged=PASS\n"
              << "future_mission_trajectory_test: PASS\n";
    return 0;
}
