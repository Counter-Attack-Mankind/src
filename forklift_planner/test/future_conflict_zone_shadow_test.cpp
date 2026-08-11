#include "forklift_planner/multi_vehicle/future_conflict_zone_shadow.h"

#include <cmath>
#include <iostream>
#include <optional>

using forklift_planner::multi_vehicle::FutureCertainty;
using forklift_planner::multi_vehicle::FutureConflictZone;
using forklift_planner::multi_vehicle::FutureConflictZoneShadowBuilder;
using forklift_planner::multi_vehicle::FutureConflictZoneSource;
using forklift_planner::multi_vehicle::FutureMissionPlanBuilder;
using forklift_planner::multi_vehicle::FutureMissionTrajectory;
using forklift_planner::multi_vehicle::FutureTrajectoryGenerator;
using forklift_planner::multi_vehicle::MissionPhase;
using forklift_planner::multi_vehicle::MultiVehicleConfig;
using forklift_planner::multi_vehicle::PathTrack;
using forklift_planner::multi_vehicle::VehicleAgent;
using forklift_planner::multi_vehicle::VehicleMode;

namespace {

int fail(const char* message) {
    std::cerr << "future_conflict_zone_shadow_test: " << message << '\n';
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

FutureMissionTrajectory makeTrajectory(
    const VehicleAgent& vehicle, double horizon,
    FutureMissionPlanBuilder& plan_builder,
    FutureTrajectoryGenerator& generator,
    const std::optional<PathTrack>& preview = std::nullopt) {
    FutureMissionTrajectory trajectory;
    trajectory.plan = plan_builder.build(vehicle, horizon, preview);
    trajectory.samples = generator.generate(trajectory.plan);
    return trajectory;
}

const FutureConflictZone* findZone(
    const std::vector<FutureConflictZone>& zones, MissionPhase phase_a,
    MissionPhase phase_b, FutureConflictZoneSource source) {
    for (const FutureConflictZone& zone : zones) {
        if (zone.phase_a == phase_a && zone.phase_b == phase_b &&
            zone.source == source) {
            return &zone;
        }
    }
    return nullptr;
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
    cfg.pickup_dwell_time = 1.0;
    cfg.unload_dwell_time = 1.0;

    FutureMissionPlanBuilder plan_builder(cfg);
    FutureTrajectoryGenerator generator(mp, cfg);
    FutureConflictZoneShadowBuilder zone_builder(mp, cfg);

    // A. Two current tracks produce CURRENT_TRACK static zones.
    VehicleAgent current_a;
    current_a.id = 0;
    current_a.mode = VehicleMode::ACTIVE;
    current_a.mission_phase = MissionPhase::TO_A1;
    current_a.path_gen = 1;
    current_a.track = line(-1.0, 0.0, 1.0, 0.0);
    VehicleAgent current_b = current_a;
    current_b.id = 1;
    current_b.track = line(0.0, -1.0, 0.0, 1.0);
    auto current_trajectory_a = makeTrajectory(
        current_a, 8.0, plan_builder, generator);
    auto current_trajectory_b = makeTrajectory(
        current_b, 8.0, plan_builder, generator);
    const auto current_zones = zone_builder.build(
        {current_trajectory_a, current_trajectory_b});
    const FutureConflictZone* current_zone = findZone(
        current_zones, MissionPhase::TO_A1, MissionPhase::TO_A1,
        FutureConflictZoneSource::CURRENT_TRACK);
    if (current_zone == nullptr || current_zone->future_zone_id < 0 ||
        current_zone->s_a_exit < current_zone->s_a_enter ||
        current_zone->s_b_exit < current_zone->s_b_enter) {
        return fail("current-track conflict zone missing");
    }

    // B. PICKUP_DWELL's prepared TO_B track conflicts with another TO_A1.
    VehicleAgent pickup;
    pickup.id = 0;
    pickup.mode = VehicleMode::DWELL;
    pickup.mission_phase = MissionPhase::PICKUP_DWELL;
    pickup.path_gen = 10;
    pickup.track = line(-1.2, 0.0, -1.0, 0.0);
    pickup.path_s = pickup.track.length();
    pickup.dwell_remaining = 1.0;
    pickup.pending_dropoff_track = line(-1.0, 0.0, 1.0, 0.0);
    pickup.pending_dropoff_valid = true;
    VehicleAgent to_a1 = current_b;
    to_a1.path_gen = 4;
    const auto pickup_trajectory = makeTrajectory(
        pickup, 10.0, plan_builder, generator);
    const auto to_a1_trajectory = makeTrajectory(
        to_a1, 10.0, plan_builder, generator);
    const auto departure_zones = zone_builder.build(
        {pickup_trajectory, to_a1_trajectory});
    const FutureConflictZone* departure_zone = findZone(
        departure_zones, MissionPhase::TO_B, MissionPhase::TO_A1,
        FutureConflictZoneSource::FUTURE_SEGMENT);
    if (departure_zone == nullptr ||
        departure_zone->path_generation_a != pickup.path_gen + 1 ||
        departure_zone->path_generation_b != to_a1.path_gen) {
        return fail("future A1 departure conflict zone missing");
    }

    // C. UNLOAD_DWELL's B->A1 preview conflicts with a current TO_B track.
    VehicleAgent unload = pickup;
    unload.mission_phase = MissionPhase::UNLOAD_DWELL;
    unload.path_gen = 20;
    unload.pending_dropoff_valid = false;
    unload.pending_dropoff_track = PathTrack{};
    const PathTrack pickup_preview = line(-1.0, 0.0, 1.0, 0.0);
    VehicleAgent to_b = current_b;
    to_b.mission_phase = MissionPhase::TO_B;
    to_b.path_gen = 6;
    const auto unload_trajectory = makeTrajectory(
        unload, 10.0, plan_builder, generator, pickup_preview);
    const auto to_b_trajectory = makeTrajectory(
        to_b, 10.0, plan_builder, generator);
    const auto return_zones = zone_builder.build(
        {unload_trajectory, to_b_trajectory});
    const FutureConflictZone* return_zone = findZone(
        return_zones, MissionPhase::TO_A1, MissionPhase::TO_B,
        FutureConflictZoneSource::FUTURE_SEGMENT);
    if (return_zone == nullptr ||
        return_zone->certainty_a != FutureCertainty::PREVIEW ||
        return_zone->path_generation_a != unload.path_gen + 1) {
        return fail("future unload-return conflict zone missing");
    }

    std::cout << "current_track_future_zone=PASS\n"
              << "pickup_to_b_x_to_a1_zone=PASS\n"
              << "unload_to_a1_x_to_b_zone=PASS\n"
              << "future_conflict_zone_shadow_test: PASS\n";
    return 0;
}
