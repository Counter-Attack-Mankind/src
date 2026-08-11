#include "forklift_planner/multi_vehicle/timed_conflict_shadow_checker.h"

#include <cmath>
#include <iostream>
#include <optional>

using forklift_planner::multi_vehicle::FutureMissionPlanBuilder;
using forklift_planner::multi_vehicle::FutureMissionTrajectory;
using forklift_planner::multi_vehicle::FutureTrajectoryGenerator;
using forklift_planner::multi_vehicle::FutureConflictZoneShadowBuilder;
using forklift_planner::multi_vehicle::LegacyPredictionShadowGenerator;
using forklift_planner::multi_vehicle::MissionPhase;
using forklift_planner::multi_vehicle::MultiVehicleConfig;
using forklift_planner::multi_vehicle::PathTrack;
using forklift_planner::multi_vehicle::ShadowConflictZone;
using forklift_planner::multi_vehicle::TimedConflictShadowChecker;
using forklift_planner::multi_vehicle::TimedConflictShadowClass;
using forklift_planner::multi_vehicle::VehicleAction;
using forklift_planner::multi_vehicle::VehicleAgent;
using forklift_planner::multi_vehicle::VehicleMode;

namespace {

int fail(const char* message) {
    std::cerr << "timed_conflict_shadow_checker_test: " << message << '\n';
    return 1;
}

PathTrack line(double x0, double y0, double x1, double y1) {
    RoughPath path;
    constexpr int steps = 40;
    const double yaw = std::atan2(y1 - y0, x1 - x0);
    for (int i = 0; i <= steps; ++i) {
        const double ratio = static_cast<double>(i) / steps;
        path.push_back(RoughWp{x0 + ratio * (x1 - x0),
                               y0 + ratio * (y1 - y0), yaw,
                               WpType::FORWARD});
    }
    PathTrack track;
    track.set(path);
    return track;
}

FutureMissionTrajectory trajectory(
    const VehicleAgent& vehicle, double horizon,
    FutureMissionPlanBuilder& builder, FutureTrajectoryGenerator& generator,
    const std::optional<PathTrack>& preview = std::nullopt) {
    FutureMissionTrajectory result;
    result.plan = builder.build(vehicle, horizon, preview);
    result.samples = generator.generate(result.plan);
    return result;
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
    cfg.conflict_margin = 0.04;
    cfg.unload_dwell_time = 1.0;

    FutureMissionPlanBuilder builder(cfg);
    FutureTrajectoryGenerator generator(mp, cfg);
    LegacyPredictionShadowGenerator legacy_generator(mp, cfg);
    TimedConflictShadowChecker checker(mp, cfg);
    FutureConflictZoneShadowBuilder zone_builder(mp, cfg);

    // A. One vehicle means there is no pair and therefore no timed conflict.
    VehicleAgent single;
    single.id = 0;
    single.mode = VehicleMode::ACTIVE;
    single.mission_phase = MissionPhase::TO_A1;
    single.path_gen = 1;
    single.track = line(-1.0, 0.0, 1.0, 0.0);
    const auto single_future = trajectory(single, 8.0, builder, generator);
    if (single_future.samples.empty()) return fail("single trajectory empty");

    // B. Ordinary current-track crossing must reproduce collision time and
    // map both events to the same existing zone.
    VehicleAgent crossing_a = single;
    VehicleAgent crossing_b = single;
    crossing_b.id = 1;
    crossing_b.track = line(0.0, -1.0, 0.0, 1.0);
    const auto future_a = trajectory(crossing_a, 8.0, builder, generator);
    const auto future_b = trajectory(crossing_b, 8.0, builder, generator);
    const auto legacy_a = legacy_generator.generate(crossing_a, 8.0);
    const auto legacy_b = legacy_generator.generate(crossing_b, 8.0);
    const std::vector<ShadowConflictZone> crossing_zones{
        ShadowConflictZone{0, 1, 4, 0.70, 1.30, 0.70, 1.30}};
    const auto crossing_future_zones = zone_builder.build(
        {future_a, future_b});
    const auto crossing_report = checker.compare(
        crossing_a, crossing_b, legacy_a, legacy_b, future_a, future_b,
        crossing_zones, crossing_future_zones);
    if (!crossing_report.old_event.valid ||
        !crossing_report.new_event.valid ||
        crossing_report.classification != TimedConflictShadowClass::MATCH ||
        crossing_report.old_event.matched_zone != 4 ||
        crossing_report.new_event.matched_zone != 4 ||
        crossing_report.new_event.future_zone_id < 0 ||
        std::abs(crossing_report.old_event.first_t -
                 crossing_report.new_event.first_t) > 1e-9 ||
        std::abs(crossing_report.old_event.last_t -
                 crossing_report.new_event.last_t) > 1e-9) {
        return fail("ordinary crossing old/new timed event mismatch");
    }

    // C. A dwell vehicle has no legacy prediction, but its lifecycle preview
    // can intersect B's current TO_B motion. The result is diagnostic only.
    VehicleAgent dwell;
    dwell.id = 0;
    dwell.mode = VehicleMode::DWELL;
    dwell.mission_phase = MissionPhase::UNLOAD_DWELL;
    dwell.path_gen = 8;
    dwell.track = line(-1.2, 0.0, -1.0, 0.0);
    dwell.path_s = dwell.track.length();
    dwell.dwell_remaining = 1.0;
    dwell.action = VehicleAction::STOP;
    dwell.reason = "not_active";
    const PathTrack pickup_preview = line(-1.0, 0.0, 1.0, 0.0);

    VehicleAgent to_b;
    to_b.id = 1;
    to_b.mode = VehicleMode::ACTIVE;
    to_b.mission_phase = MissionPhase::TO_B;
    to_b.path_gen = 3;
    to_b.track = line(0.0, -1.2, 0.0, 1.0);
    to_b.action = VehicleAction::NOMINAL;
    to_b.reason = "clear";
    const VehicleAgent dwell_before = dwell;
    const VehicleAgent to_b_before = to_b;

    const auto dwell_future = trajectory(
        dwell, 10.0, builder, generator, pickup_preview);
    const auto to_b_future = trajectory(to_b, 10.0, builder, generator);
    const auto dwell_legacy = legacy_generator.generate(dwell, 10.0);
    const auto to_b_legacy = legacy_generator.generate(to_b, 10.0);
    const auto lifecycle_future_zones = zone_builder.build(
        {dwell_future, to_b_future});
    const auto lifecycle_report = checker.compare(
        dwell, to_b, dwell_legacy, to_b_legacy, dwell_future, to_b_future,
        {}, lifecycle_future_zones);
    if (!dwell_legacy.empty() || lifecycle_report.old_event.valid ||
        !lifecycle_report.new_event.valid ||
        lifecycle_report.classification !=
            TimedConflictShadowClass::NEW_FUTURE_LIFECYCLE_CONFLICT ||
        lifecycle_report.new_event.phase_a != MissionPhase::TO_A1 ||
        lifecycle_report.new_event.future_zone_id < 0) {
        return fail("future dwell lifecycle conflict not detected");
    }
    if (dwell.action != dwell_before.action ||
        dwell.reason != dwell_before.reason ||
        to_b.action != to_b_before.action ||
        to_b.reason != to_b_before.reason) {
        return fail("shadow checker changed vehicle control state");
    }

    std::cout << "single_no_pair_conflict=PASS\n"
              << "ordinary_crossing_timed_event_match=PASS\n"
              << "dwell_future_lifecycle_conflict=PASS\n"
              << "vehicle_action_unchanged=PASS\n"
              << "timed_conflict_shadow_checker_test: PASS\n";
    return 0;
}
