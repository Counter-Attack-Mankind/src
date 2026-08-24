#include "forklift_planner/multi_vehicle/shared_segment_geometry.h"

#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "forklift_planner/multi_vehicle/footprint.h"

using forklift_planner::multi_vehicle::MultiVehicleConfig;
using forklift_planner::multi_vehicle::PathTrack;
using forklift_planner::multi_vehicle::SharedSegmentCandidate;
using forklift_planner::multi_vehicle::computeSharedSegmentCandidates;
using forklift_planner::multi_vehicle::makeBody;
using forklift_planner::multi_vehicle::overlaps;

namespace {

constexpr double kPi = 3.14159265358979323846;

int fail(const std::string& message) {
    std::cerr << "shared_segment_geometry_test: " << message << '\n';
    return 1;
}

RoughWp wp(double x, double y, double theta, WpType type) {
    return RoughWp{x, y, theta, type};
}

PathTrack track(const RoughPath& path) {
    PathTrack output;
    output.set(path);
    return output;
}

RoughPath line(double x0, double x1, double y, double theta,
               WpType type, int count = 41) {
    RoughPath path;
    path.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double ratio = static_cast<double>(i) / (count - 1);
        path.push_back(wp(x0 + (x1 - x0) * ratio, y, theta, type));
    }
    return path;
}

std::vector<SharedSegmentCandidate> candidates(
    const RoughPath& a, const RoughPath& b) {
    MapParam mp;
    MultiVehicleConfig cfg;
    cfg.conflict_margin = 0.04;
    // Structural tests below exercise traversal and connectivity independently
    // of the component-level qualification added after closure.
    cfg.shared_segment_min_span = 0.0;
    cfg.shared_segment_min_strong_ratio = 0.0;
    const PathTrack ta = track(a);
    const PathTrack tb = track(b);
    return computeSharedSegmentCandidates(ta, 11, tb, 22, mp, cfg);
}

bool hasTraversalPair(const std::vector<SharedSegmentCandidate>& values,
                      int traversal_a, int traversal_b) {
    for (const auto& value : values) {
        if (value.traversal_a == traversal_a &&
            value.traversal_b == traversal_b) {
            return true;
        }
    }
    return false;
}

int testStraightOpposing() {
    const auto values = candidates(
        line(-0.50, 0.50, 0.0, 0.0, WpType::FORWARD),
        line(0.50, -0.50, 0.0, 0.0, WpType::REVERSE));
    if (values.size() != 1) return fail("Test 1 expected one candidate");
    const auto& c = values.front();
    if (c.direction_a != WpType::FORWARD ||
        c.direction_b != WpType::REVERSE || c.sample_count == 0 ||
        c.direction_dot_max >= -0.5 || c.path_gen_a != 11 ||
        c.path_gen_b != 22) {
        return fail("Test 1 candidate fields are wrong");
    }
    return 0;
}

int testCuspSplit() {
    RoughPath a = line(-0.50, 0.0, 0.0, kPi, WpType::REVERSE, 21);
    const RoughPath forward =
        line(0.0, 0.50, 0.0, 0.0, WpType::FORWARD, 21);
    a.insert(a.end(), forward.begin(), forward.end());
    const RoughPath b = line(0.50, -0.50, 0.0, 0.0, WpType::REVERSE);
    const auto values = candidates(a, b);
    if (!hasTraversalPair(values, 0, 0) ||
        !hasTraversalPair(values, 1, 0)) {
        return fail("Test 2 did not split R/R from F/R at the cusp");
    }
    return 0;
}

int testRepeatedWorldSpace() {
    RoughPath a = line(-0.50, 0.50, 0.0, kPi, WpType::REVERSE);
    const RoughPath a_return =
        line(0.50, -0.50, 0.0, kPi, WpType::FORWARD);
    a.insert(a.end(), a_return.begin(), a_return.end());
    RoughPath b = line(0.50, -0.50, 0.0, 0.0, WpType::REVERSE);
    const RoughPath b_return =
        line(-0.50, 0.50, 0.0, 0.0, WpType::FORWARD);
    b.insert(b.end(), b_return.begin(), b_return.end());
    const auto values = candidates(a, b);
    if (!hasTraversalPair(values, 0, 0) ||
        !hasTraversalPair(values, 1, 1)) {
        return fail("Test 3 merged repeated world-space traversals");
    }
    return 0;
}

int testForwardTurnDoesNotSplitTraversal() {
    RoughPath a;
    RoughPath b;
    for (int i = 0; i <= 20; ++i) {
        const double r = static_cast<double>(i) / 20.0;
        a.push_back(wp(-0.50 + 0.50 * r, 0.0, 0.0, WpType::FORWARD));
        b.push_back(wp(0.0, 0.50 - 0.50 * r, -0.5 * kPi,
                       WpType::FORWARD));
    }
    for (int i = 1; i <= 20; ++i) {
        const double r = static_cast<double>(i) / 20.0;
        a.push_back(wp(0.0, 0.50 * r, 0.5 * kPi, WpType::FORWARD));
        b.push_back(wp(-0.50 * r, 0.0, kPi, WpType::FORWARD));
    }
    const auto values = candidates(a, b);
    if (values.empty()) return fail("Test 4 produced no turning candidate");
    for (const auto& value : values) {
        if (value.traversal_a != 0 || value.traversal_b != 0) {
            return fail("Test 4 split a continuous FORWARD turn");
        }
    }
    return 0;
}

int testSemanticGapSplitsComponents() {
    RoughPath a = line(-0.60, 0.60, 0.0, 0.0, WpType::FORWARD, 97);
    RoughPath b;
    b.reserve(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double ratio = static_cast<double>(i) / (a.size() - 1);
        const double x = 0.60 - 1.20 * ratio;
        double theta = kPi;
        if (ratio > 0.38 && ratio < 0.62) theta = 0.0;
        b.push_back(wp(x, 0.0, theta, WpType::FORWARD));
    }
    const auto values = candidates(a, b);
    if (values.size() < 2) {
        return fail("Test 5 bridged an opposing/non-opposing semantic gap");
    }
    for (const auto& value : values) {
        if (value.traversal_a != 0 || value.traversal_b != 0) {
            return fail("Test 5 unexpectedly changed traversal identity");
        }
    }
    return 0;
}

int testReverseTurnUsesLocalHeading() {
    RoughPath a;
    RoughPath b;
    for (int i = 0; i <= 60; ++i) {
        const double u = static_cast<double>(i) / 60.0;
        const double angle = -0.25 * kPi + 0.50 * kPi * u;
        const double x = 0.40 * std::sin(angle);
        const double y = 0.40 * (1.0 - std::cos(angle));
        const double tangent = angle;
        a.push_back(wp(x, y, tangent + kPi, WpType::REVERSE));
        b.push_back(wp(-x, y, tangent, WpType::REVERSE));
    }
    const auto values = candidates(a, b);
    if (values.empty()) return fail("Test 6 produced no reverse-turn candidate");
    bool varying_dot = false;
    for (const auto& value : values) {
        if (value.direction_a != WpType::REVERSE ||
            value.direction_b != WpType::REVERSE) {
            return fail("Test 6 lost REVERSE traversal identity");
        }
        varying_dot = varying_dot ||
            value.direction_dot_max - value.direction_dot_min > 1e-3;
    }
    if (!varying_dot) {
        return fail("Test 6 did not evaluate motion heading per sample");
    }
    return 0;
}

int testNonOpposingOverlapRejected() {
    const RoughPath a = line(-0.50, 0.50, 0.0, 0.0, WpType::FORWARD);
    const RoughPath b = line(-0.50, 0.50, 0.0, 0.0, WpType::FORWARD);
    MapParam mp;
    MultiVehicleConfig cfg;
    cfg.conflict_margin = 0.04;
    const PathTrack ta = track(a);
    const PathTrack tb = track(b);
    if (!overlaps(makeBody(ta.poseAtS(0.5), mp, 0.02),
                  makeBody(tb.poseAtS(0.5), mp, 0.02))) {
        return fail("Test 7 setup does not contain geometric OBB overlap");
    }
    if (!computeSharedSegmentCandidates(ta, 11, tb, 22, mp, cfg).empty()) {
        return fail("Test 7 accepted non-opposing overlap");
    }
    return 0;
}

int testDeterministicIds() {
    const RoughPath a = line(-0.50, 0.50, 0.0, 0.0, WpType::FORWARD);
    const RoughPath b = line(0.50, -0.50, 0.0, kPi, WpType::FORWARD);
    const auto first = candidates(a, b);
    const auto second = candidates(a, b);
    if (first.size() != second.size()) return fail("determinism size mismatch");
    for (std::size_t i = 0; i < first.size(); ++i) {
        if (first[i].id != static_cast<int>(i) ||
            first[i].id != second[i].id ||
            first[i].traversal_a != second[i].traversal_a ||
            first[i].traversal_b != second[i].traversal_b ||
            std::abs(first[i].s_a_enter - second[i].s_a_enter) > 1e-12 ||
            std::abs(first[i].s_b_enter - second[i].s_b_enter) > 1e-12) {
            return fail("candidate IDs are not deterministic");
        }
    }
    return 0;
}

int testComponentQualification() {
    MapParam mp;
    MultiVehicleConfig cfg;
    cfg.conflict_margin = 0.04;
    cfg.shared_segment_min_span = 0.50;
    cfg.shared_segment_strong_opposing_threshold = -0.80;
    cfg.shared_segment_min_strong_ratio = 0.60;

    const PathTrack stable_a = track(
        line(-0.60, 0.60, 0.0, 0.0, WpType::FORWARD, 97));
    const PathTrack stable_b = track(
        line(0.60, -0.60, 0.0, kPi, WpType::FORWARD, 97));
    const auto stable = computeSharedSegmentCandidates(
        stable_a, 11, stable_b, 22, mp, cfg);
    if (stable.size() != 1 ||
        std::abs(stable.front().direction_dot_mean + 1.0) > 1e-9 ||
        stable.front().strong_opposing_count != stable.front().sample_count ||
        std::abs(stable.front().strong_opposing_ratio - 1.0) > 1e-12) {
        return fail("qualification rejected a stable opposing segment");
    }

    const PathTrack short_a = track(
        line(-0.15, 0.15, 0.0, 0.0, WpType::FORWARD, 25));
    const PathTrack short_b = track(
        line(0.15, -0.15, 0.0, kPi, WpType::FORWARD, 25));
    if (!computeSharedSegmentCandidates(
             short_a, 11, short_b, 22, mp, cfg).empty()) {
        return fail("qualification accepted a short opposing component");
    }
    MultiVehicleConfig unfiltered = cfg;
    unfiltered.shared_segment_min_span = 0.0;
    unfiltered.shared_segment_min_strong_ratio = 0.0;
    if (computeSharedSegmentCandidates(
            short_a, 11, short_b, 22, mp, unfiltered).empty()) {
        return fail("short-component setup produced no opposing component");
    }

    const double weak_opposing_heading = std::acos(-0.60);
    const PathTrack weak_a = track(
        line(-0.60, 0.60, 0.0, 0.0, WpType::FORWARD, 97));
    const PathTrack weak_b = track(
        line(0.60, -0.60, 0.0, weak_opposing_heading,
             WpType::FORWARD, 97));
    const auto weak_component = computeSharedSegmentCandidates(
        weak_a, 11, weak_b, 22, mp, unfiltered);
    if (weak_component.size() != 1 ||
        weak_component.front().strong_opposing_count != 0 ||
        std::abs(weak_component.front().direction_dot_mean + 0.60) > 1e-9) {
        return fail("weak-opposing distribution setup is invalid");
    }
    if (!computeSharedSegmentCandidates(
             weak_a, 11, weak_b, 22, mp, cfg).empty()) {
        return fail("qualification accepted a weak-opposing component");
    }
    return 0;
}

}  // namespace

int main() {
    for (int (*test)() : {testStraightOpposing, testCuspSplit,
                          testRepeatedWorldSpace,
                          testForwardTurnDoesNotSplitTraversal,
                          testSemanticGapSplitsComponents,
                          testReverseTurnUsesLocalHeading,
                          testNonOpposingOverlapRejected,
                          testDeterministicIds,
                          testComponentQualification}) {
        if (const int result = test()) return result;
    }
    std::cout << "shared_segment_geometry_test: PASS\n";
    return 0;
}
