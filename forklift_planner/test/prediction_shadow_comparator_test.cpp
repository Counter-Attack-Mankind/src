#include "forklift_planner/multi_vehicle/prediction_shadow_comparator.h"

#include <cmath>
#include <iostream>

using forklift_planner::multi_vehicle::FutureMissionPlanBuilder;
using forklift_planner::multi_vehicle::FutureMissionTrajectory;
using forklift_planner::multi_vehicle::FutureTrajectoryGenerator;
using forklift_planner::multi_vehicle::LegacyPredictionShadowGenerator;
using forklift_planner::multi_vehicle::MissionPhase;
using forklift_planner::multi_vehicle::MultiVehicleConfig;
using forklift_planner::multi_vehicle::PathTrack;
using forklift_planner::multi_vehicle::PredictionShadowComparator;
using forklift_planner::multi_vehicle::ShadowMismatchKind;
using forklift_planner::multi_vehicle::VehicleAgent;
using forklift_planner::multi_vehicle::VehicleMode;

namespace {

int fail(const char* message) {
    std::cerr << "prediction_shadow_comparator_test: " << message << '\n';
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

FutureMissionTrajectory buildTrajectory(
    const VehicleAgent& vehicle, double horizon,
    FutureMissionPlanBuilder& builder, FutureTrajectoryGenerator& generator) {
    FutureMissionTrajectory result;
    result.plan = builder.build(vehicle, horizon);
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
    cfg.conflict_margin = 0.12;
    cfg.pickup_dwell_time = 5.0;

    FutureMissionPlanBuilder builder(cfg);
    FutureTrajectoryGenerator generator(mp, cfg);
    LegacyPredictionShadowGenerator legacy_generator(mp, cfg);
    PredictionShadowComparator comparator;

    // A. Over the current motion segment both integrations are identical.
    VehicleAgent current;
    current.id = 0;
    current.mode = VehicleMode::ACTIVE;
    current.mission_phase = MissionPhase::TO_A1;
    current.path_gen = 4;
    current.track = straight(0.0, 10.0);
    const auto current_future =
        buildTrajectory(current, 15.0, builder, generator);
    const auto current_legacy = legacy_generator.generate(current, 15.0);
    const auto current_report =
        comparator.compare(current, current_legacy, current_future);
    if (current_report.matched_sample_count != current_future.samples.size() ||
        current_report.segment_mismatch_count != 0 ||
        current_report.time_mismatch_count != 0 ||
        current_report.max_s_error > 1e-12 ||
        current_report.max_position_error > 1e-12 ||
        current_report.max_speed_error > 1e-12 ||
        current_report.max_body_center_error > 1e-12 ||
        current_report.max_body_yaw_error > 1e-12) {
        std::cerr << "old=" << current_report.old_sample_count
                  << " new=" << current_report.new_sample_count
                  << " matched=" << current_report.matched_sample_count
                  << " segment_mismatch="
                  << current_report.segment_mismatch_count
                  << " time_mismatch=" << current_report.time_mismatch_count
                  << " max_s=" << current_report.max_s_error
                  << " max_position=" << current_report.max_position_error
                  << " max_speed=" << current_report.max_speed_error
                  << " max_body_center="
                  << current_report.max_body_center_error
                  << " max_body_yaw=" << current_report.max_body_yaw_error
                  << '\n';
        return fail("current motion segment did not reproduce legacy prediction");
    }

    // B. Crossing into dwell/TO_B is classified as lifecycle mismatch while
    // the preceding current-track samples remain numerically identical.
    VehicleAgent lifecycle = current;
    lifecycle.id = 1;
    lifecycle.track = straight(0.0, 0.8);
    lifecycle.pending_dropoff_track = straight(0.8, 1.6);
    lifecycle.pending_dropoff_valid = true;
    const auto lifecycle_future =
        buildTrajectory(lifecycle, 15.0, builder, generator);
    const auto lifecycle_legacy = legacy_generator.generate(lifecycle, 15.0);
    const auto lifecycle_report =
        comparator.compare(lifecycle, lifecycle_legacy, lifecycle_future);
    if (lifecycle_report.matched_sample_count == 0 ||
        lifecycle_report.segment_mismatch_count == 0 ||
        lifecycle_report.max_s_error > 1e-12 ||
        !lifecycle_report.first_mismatch.valid ||
        lifecycle_report.first_mismatch.mismatch !=
            ShadowMismatchKind::FUTURE_LIFECYCLE_SEGMENT) {
        return fail("lifecycle transition was not separated from numeric error");
    }

    // C. RuleEngine does not run its local predictor for DWELL vehicles. The
    // comparator must report unavailable-old, not a motion-model error.
    VehicleAgent dwell = lifecycle;
    dwell.id = 2;
    dwell.mode = VehicleMode::DWELL;
    dwell.mission_phase = MissionPhase::PICKUP_DWELL;
    dwell.dwell_remaining = 2.0;
    const auto dwell_future = buildTrajectory(dwell, 15.0, builder, generator);
    const auto dwell_legacy = legacy_generator.generate(dwell, 15.0);
    const auto dwell_report = comparator.compare(dwell, dwell_legacy,
                                                 dwell_future);
    if (!dwell_legacy.empty() || dwell_report.matched_sample_count != 0 ||
        dwell_report.segment_mismatch_count != dwell_future.samples.size() ||
        !dwell_report.first_mismatch.valid ||
        dwell_report.first_mismatch.mismatch !=
            ShadowMismatchKind::OLD_PREDICTION_UNAVAILABLE) {
        return fail("DWELL old-prediction absence was classified incorrectly");
    }

    std::cout << "current_segment_numeric_match=PASS\n"
              << "lifecycle_segment_classification=PASS\n"
              << "dwell_old_unavailable_classification=PASS\n"
              << "prediction_shadow_comparator_test: PASS\n";
    return 0;
}
