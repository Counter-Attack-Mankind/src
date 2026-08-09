#include "forklift_planner/multi_vehicle/future_a1_policy.h"

#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

using forklift_planner::multi_vehicle::FutureA1RankedCandidate;
using forklift_planner::multi_vehicle::futureA1ArrivalWithinHorizon;
using forklift_planner::multi_vehicle::futureA1StopS;
using forklift_planner::multi_vehicle::futureA1TransitionReason;
using forklift_planner::multi_vehicle::selectFutureA1Candidate;

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

int fail(const char* message) {
    std::cerr << "future_a1_policy_test: " << message << '\n';
    return 1;
}

}  // namespace

int main() {
    // A. No arrival outside the active horizon may become a candidate.
    if (futureA1ArrivalWithinHorizon(15.1, 15.0) ||
        futureA1ArrivalWithinHorizon(41.0, 15.0)) {
        return fail("arrival beyond horizon was accepted");
    }

    // B. A clearly earlier new candidate replaces the old owner.
    const std::vector<FutureA1RankedCandidate> candidates{
        {0, 14.0}, {1, 6.0}};
    const int selected = selectFutureA1Candidate(
        candidates, 0.2, [](int lhs, int rhs) {
            return std::min(lhs, rhs);
        });
    if (selected != 1) return fail("earlier candidate did not win");
    if (futureA1TransitionReason(
            true, 0, true, selected, false, true, false, false) !=
        "earlier_candidate") {
        return fail("owner change was not classified as earlier_candidate");
    }
    std::cout << "B event=CHANGE old=V0 new=V1 "
                 "change_reason=earlier_candidate\n";

    // C. Admission must stop before the most-upstream relevant entry.
    const std::optional<double> stop_s = futureA1StopS(3.075, 2.175, 0.01);
    if (!stop_s || !near(*stop_s, 2.165) || *stop_s >= 2.175) {
        return fail("ordinary conflict entry did not bound stop_s");
    }

    std::cout << "A no_owner_when_all_eta_exceed_horizon=PASS\n"
              << "C future_exit_enter=3.075 ordinary_enter=2.175 "
                 "stop_s=2.165 PASS\n"
              << "future_a1_policy_test: PASS\n";
    return 0;
}
