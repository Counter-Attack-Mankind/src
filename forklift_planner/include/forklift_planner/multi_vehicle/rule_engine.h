#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/dynamic_speed_coordination.h"
#include "forklift_planner/multi_vehicle/future_a1_policy.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/spatiotemporal_interaction.h"
#include "forklift_planner/multi_vehicle/traffic_resource.h"
#include "forklift_planner/multi_vehicle/traffic_resource_map.h"
#include "forklift_planner/multi_vehicle/vehicle_agent.h"

namespace forklift_planner {
namespace multi_vehicle {

enum class ConflictMarkerKind {
    SAME_DIRECTION,
    CROSSING_OR_OPPOSING,
    POTENTIAL_CONFLICT_ZONE,
    CONFLICT_RESERVATION,
};

struct ConflictMarker {
    using Point = InteractionPoint;
    using TimedOverlap = TimedOverlapGeometry;

    // For CROSSING_OR_OPPOSING, x/y and scale_x/scale_y are the AABB around
    // sampled, time-synchronised OBB intersections. For the two resource-only
    // marker kinds they are a label anchor and diagnostic bounds. None of
    // these display fields participates in arbitration.
    double x = 0.0;
    double y = 0.0;
    double t = 0.0;
    double scale_x = 0.08;
    double scale_y = 0.08;
    int vehicle_a = -1;
    int vehicle_b = -1;
    int follower_id = -1;
    int leader_id = -1;
    int holder_id = -1;
    int waiter_id = -1;
    double following_gap = 0.0;
    VehicleAction following_action = VehicleAction::NOMINAL;
    double follower_x = 0.0;
    double follower_y = 0.0;
    double leader_x = 0.0;
    double leader_y = 0.0;
    std::vector<TimedOverlap> timed_overlaps;
    // Static, time-independent OBB intersections reconstructed from one
    // ConflictZone/reservation arc-length rectangle. Kept separate from
    // timed_overlaps so RViz cannot confuse potential/locked resources with
    // a time-synchronised predicted collision.
    std::vector<std::vector<Point>> spatial_overlap_polygons;
    // Diagnostic identity only. raw_zone_index is stable for one
    // (vehicle pair, path generation pair); active_zone_index is the current
    // findConflictZones() vector position after cleared zones are filtered.
    int raw_zone_index = -1;
    int active_zone_index = -1;
    int path_gen_a = -1;
    int path_gen_b = -1;
    bool zone_aabb_valid = false;
    double s_a_enter = 0.0;
    double s_a_exit = 0.0;
    double s_b_enter = 0.0;
    double s_b_exit = 0.0;
    ConflictMarkerKind kind = ConflictMarkerKind::CROSSING_OR_OPPOSING;
};

class RuleEngine {
public:
    RuleEngine(const MapParam& mp, const MultiVehicleConfig& cfg);

    struct FutureA1Commitment {
        int owner_id = -1;
        int owner_path_gen = -1;
        double predicted_a1_arrival_time = -1.0;
        double predicted_to_b_time = -1.0;

        bool valid() const {
            return owner_id >= 0 && owner_path_gen >= 0 &&
                   predicted_a1_arrival_time >= 0.0 &&
                   predicted_to_b_time >= predicted_a1_arrival_time;
        }
    };

    void setFutureA1Commitment(const FutureA1Commitment& commitment) {
        if (commitment.owner_id != future_a1_admission_log_owner_id_ ||
            commitment.owner_path_gen !=
                future_a1_admission_log_owner_path_gen_) {
            future_a1_admission_logged_.clear();
            future_a1_admission_log_owner_id_ = commitment.owner_id;
            future_a1_admission_log_owner_path_gen_ =
                commitment.owner_path_gen;
        }
        future_a1_commitment_ = commitment;
    }
    void clearFutureA1Commitment() {
        future_a1_commitment_ = FutureA1Commitment{};
    }

    void setCoordLogSink(const std::function<void(const std::string&)>& sink) {
        coord_log_sink_ = sink;
    }
    // Diagnostic metadata only. It never participates in a rule decision.
    void setDebugLogContext(const std::string& source, uint64_t plan_id,
                            int frame_id, int rollout_step) {
        if (source != debug_log_source_ || plan_id != debug_log_plan_id_) {
            pairwise_conflict_logs_.clear();
            a1_decision_logs_.clear();
        }
        debug_log_source_ = source;
        debug_log_plan_id_ = plan_id;
        debug_log_frame_id_ = frame_id;
        debug_log_rollout_step_ = rollout_step;
    }

    // Reservation for one visible spatiotemporal conflict event. Arc ranges
    // use canonical vehicle-id order (lo, hi).
    struct ConflictReservation {
        int owner_id = -1;
        int gen_lo = -1;
        int gen_hi = -1;
        double enter_lo = 0.0;
        double exit_lo = 0.0;
        double enter_hi = 0.0;
        double exit_hi = 0.0;
        double x = 0.0;
        double y = 0.0;
        double first_conflict_t = 0.0;
        // RViz/debug metadata. These fields never participate in arbitration.
        int raw_zone_index = -1;
        double aabb_min_x = 0.0;
        double aabb_min_y = 0.0;
        double aabb_max_x = 0.0;
        double aabb_max_y = 0.0;
        bool aabb_valid = false;
    };

    // Multi-zone A1 departure handoff. Unlike ConflictReservation, this
    // protects one transitive conflict cluster rather than one timed event.
    // A staged entry is produced from Future A1 geometry with the exact
    // generation that activatePreparedDropoffLeg() will assign; it becomes
    // active only after that owner actually enters TO_B.
    struct DepartureClusterCommitment {
        int owner_id = -1;
        int owner_path_gen = -1;
        int other_id = -1;
        int other_path_gen = -1;
        std::vector<size_t> seed_indices;
        std::vector<size_t> cluster_indices;
        std::vector<FutureA1ConflictInterval> intervals;
        double waiter_stop_boundary_s = 0.0;
        double waiter_stop_s = 0.0;
        double owner_release_exit_s = 0.0;
        double other_release_exit_s = 0.0;
        bool active = false;
        bool handed_off_from_future = false;
        bool handoff_already_inside = false;
        bool hold_logged = false;
    };

    // 接入资源地图(Phase 2:资源仲裁需要它把路径映射到资源占用)。
    void setResourceMap(const TrafficResourceMap* m) { resmap_ = m; }

    // prediction_horizon_override >= 0 constrains this decision to the
    // remaining part of one already-active planning window. Normal callers
    // keep the configured horizon by omitting it.
    void decide(std::vector<VehicleAgent>& vehicles, double dt,
                double prediction_horizon_override = -1.0);
    double speedForAction(VehicleAction action) const;

    // 前瞻仿真用:快照/恢复跨周期持久状态,使「克隆-空跑」忠实复现真实协调(确定性⇒预测准)。
    struct SimSnapshot {
        std::map<std::pair<int, int>, ConflictReservation> reservations;
        std::map<std::pair<int, int>, DepartureClusterCommitment>
            departure_clusters;
        std::set<std::pair<int, int>> following_pairs;
        ResourceTokenTable tokens;
        // conflicts_ is frame output, but sandbox rollout calls decide() and
        // rewrites it on every predicted step. Snapshot it as well so a
        // rollout cannot leak its final predicted frame into current RViz.
        std::vector<ConflictMarker> conflicts;
        double now = 0.0;
    };
    SimSnapshot snapshot() const;
    void restore(const SimSnapshot& s);
    // Read-only stress-test diagnostics. This exposes the current commitment
    // without creating, changing, or releasing any coordination state.
    const FutureA1Commitment& futureA1Commitment() const {
        return future_a1_commitment_;
    }
    const std::vector<ConflictMarker>& conflicts() const { return conflicts_; }
    // RViz-only resource diagnostics. This reads the current static conflict
    // cache and reservation table; it never creates, updates or releases a
    // reservation and is intentionally outside decide().
    std::vector<ConflictMarker> conflictResourceMarkers(
        const std::vector<VehicleAgent>& vehicles) const;

    int priorityWinner(const VehicleAgent& a, const VehicleAgent& b) const;

    int unifiedPriority(const VehicleAgent& a, const VehicleAgent& b) const;

    struct DynamicSpeedMetrics {
        unsigned long long baseline_conflicts = 0;
        unsigned long long far_deferred = 0;
        unsigned long long mid_interventions = 0;
        unsigned long long near_interventions = 0;
        unsigned long long near_legacy_fallbacks = 0;
        unsigned long long yield_trials = 0;
        unsigned long long yield_clear = 0;
        unsigned long long creep_trials = 0;
        unsigned long long creep_clear = 0;
        unsigned long long candidate_search_failed = 0;
        unsigned long long near_fallbacks = 0;
        unsigned long long a1_fallbacks = 0;
        unsigned long long existing_reservation_skips = 0;
        unsigned long long nominal_recoveries = 0;
        unsigned long long reservation_creates = 0;
        unsigned long long reservation_deletes = 0;
    };
    const DynamicSpeedMetrics& dynamicSpeedMetrics() const {
        return dynamic_speed_metrics_;
    }

    // Read-only coordination interface. The path_gen-keyed geometry cache is
    // the only mutable implementation detail this may populate.
    PairInteractionResult detectPairInteraction(
        const VehicleAgent& a, const VehicleAgent& b,
        double prediction_horizon) const;

    // 诊断:打印一对车的冲突区全貌(各块 se/sx、same_dir、committed、停止线、owner 预约、
    // 是否在 following_pairs_)。供无头批处理在碰撞现场调用,定位"该不该门控/谁越线"。
    void debugDumpConflict(const VehicleAgent& a, const VehicleAgent& b) const;

private:
    using ConflictZone = PotentialConflictZone;

    struct FutureA1ZoneSelection {
        std::vector<ConflictZone> normalized_zones;
        std::vector<size_t> seed_indices;
        std::vector<size_t> protected_indices;
        int upstream_index = -1;
        bool other_already_inside = false;
    };

    FutureA1ZoneSelection selectFutureA1ProtectedZones(
        const std::vector<ConflictZone>& canonical_zones,
        bool preview_is_lo, double protected_until,
        double other_path_s) const;

    // One-cycle longitudinal hint. It is intentionally separate from
    // VehicleAgent::requested_action because following is not an arbitration
    // authority: pairwise reservations/priority and all later safety rules
    // must be able to decide first.
    struct FollowingSuggestion {
        int follower_id = -1;
        int leader_id = -1;
        VehicleAction action = VehicleAction::NOMINAL;
        double gap = 0.0;
    };

    // 当前位置下、属于 (self,other) 的有效冲突块:取缓存的静态 C_ij(见
    // conflictBlocksCanonical),按 self/other 朝向取用,并裁掉任一方已完全清出的块、
    // 把入口夹到各自车尾起点——与历史"逐拍沿剩余路径扫描"的产物在同一离散精度下等价。
    std::vector<ConflictZone> findConflictZones(const VehicleAgent& self,
                                                const VehicleAgent& other) const;
    // 在两条「完全固定」路径的整段 [0,L]×[0,L] 上一次算定的静态冲突集 C_ij(与
    // 时间/速度/朝向/当前位置无关)。仅几何,重活在此。
    std::vector<ConflictZone> computeConflictZonesFull(
        const VehicleAgent& self, const VehicleAgent& other) const;
    // 取 (lo,hi)(lo.id<hi.id)的静态 C_ij,按 path_gen 缓存;任一方换路径才重算。
    // 返回的块以 self=lo、other=hi 的朝向存储(s_self_*=lo 路径弧长)。
    const std::vector<ConflictZone>& conflictBlocksCanonical(
        const VehicleAgent& lo, const VehicleAgent& hi) const;
    void recordConflictZones(const VehicleAgent& self,
                             const VehicleAgent& other,
                             const std::vector<ConflictZone>& zones,
                             ConflictMarkerKind kind,
                             double first_conflict_t = 0.0,
                             int follower_id = -1,
                             int leader_id = -1,
                             double following_gap = 0.0,
                             VehicleAction following_action =
                                 VehicleAction::NOMINAL,
                             int holder_id = -1,
                             int waiter_id = -1,
                             const std::vector<ConflictMarker::TimedOverlap>&
                                 timed_overlaps = {});
    double timeToReachS(const VehicleAgent& v, VehicleAction action,
                        double target_s) const;
    double predictedTravelDistance(const VehicleAgent& v,
                                   VehicleAction action,
                                   double t) const;
    void applyActionRequest(VehicleAgent& v, VehicleAction action,
                            const std::string& reason, int blocker_id = -1);
    void resolvePairwiseConflicts(std::vector<VehicleAgent>& vehicles, double dt,
                                  double prediction_horizon);
    void enforceFutureA1Admission(std::vector<VehicleAgent>& vehicles,
                                  double dt);
    void refreshDepartureClusterCommitments(
        std::vector<VehicleAgent>& vehicles);
    void enforceDepartureClusterCommitments(
        std::vector<VehicleAgent>& vehicles, double dt);
    int departureClusterOwnerForPair(const VehicleAgent& a,
                                     const VehicleAgent& b) const;
    void resolveFollowing(std::vector<VehicleAgent>& vehicles);
    void applyFollowingSuggestions(std::vector<VehicleAgent>& vehicles);
    // 普适前向净空护栏:任何车若沿自身固定路径在自己刹车距离内会撞上另一辆车的当前
    // 车身,提前 STOP(留余量、干净对停)。堵死 following/crossing 分类接缝处「两套都
    // 没刹→NOMINAL 直撞停着的车→十字楔死」的漏洞。破环车豁免。比硬护栏早刹留余量。
    void enforceForwardClearance(std::vector<VehicleAgent>& vehicles);
    void resolveTargetSlotOccupancy(std::vector<VehicleAgent>& vehicles);
    void applyRequestedActions(std::vector<VehicleAgent>& vehicles, double dt);
    void breakDeadlockCycles(std::vector<VehicleAgent>& vehicles);
    // Phase 4(§9/§11.11):用上一周期的等待边(blocker_id)建等待图,检测环,
    // 按 §9 顺序选破环车并置 deadlock_breaker(资源/优先级层据此给它临时最高优先级)。
    void resolveDeadlock(std::vector<VehicleAgent>& vehicles, double dt);
    // Phase 2:track 变化时,用资源地图重算每车路径的资源占用区间(缓存到 agent)。
    void refreshResourceSpans(std::vector<VehicleAgent>& vehicles);
    // Phase 2.2:对 capacity=1 互斥资源(窄道/路口/货位口)按统一优先级发令牌,
    // winner 通过、其余在上游停止线让行;令牌持有到车身驶出才释放(§11)。
    void arbitrateResources(std::vector<VehicleAgent>& vehicles, double dt);
    std::string debugLogPrefix() const;

    const MapParam& mp_;
    const MultiVehicleConfig& cfg_;
    std::vector<ConflictMarker> conflicts_;
    // Diagnostic de-duplication only: one pair/holder event per plan context.
    std::set<std::tuple<int, int, int>> pairwise_conflict_logs_;
    std::set<std::tuple<int, int, int, int>> a1_decision_logs_;
    bool shouldLogA1Decision(const VehicleAgent& vehicle, int blocker_id);

    // 未来时域内最早可见的局部冲突事件预约。它只锁定本次事件对应的弧长区间，
    // 不再把同一车对完整路径上的所有几何交叉合并为一个大资源。
    std::map<std::pair<int, int>, ConflictReservation> conflict_reservations_;
    std::map<std::pair<int, int>, DepartureClusterCommitment>
        departure_cluster_commitments_;

    // 每周期重算的唯一有向跟车对，仅作诊断/纵向建议依据。
    // pairwise timed OBB、reservation 和 holder 仲裁不再使用它作跳过条件。
    std::set<std::pair<int, int>> following_pairs_;
    std::map<std::pair<int, int>, FollowingSuggestion> following_suggestions_;
    // Reconstructed before each reset from the live vehicle reason. This is
    // diagnostic-only and never participates in coordination.
    std::map<std::pair<int, int>, VehicleAction> previous_dynamic_actions_;
    // Directed following actions applied in the preceding decision, captured
    // before VehicleAgent::reason is reset. Used only to release a stale
    // following action_hold when pairwise now grants that follower passage.
    std::map<std::pair<int, int>, int> previous_following_followers_;
    // Pairs for which reservation/OBB arbitration supplied the result in this
    // decision. A following hint must not be applied to those pairs.
    std::set<std::pair<int, int>> pairwise_managed_pairs_;

    // 静态冲突集 C_ij 缓存(协调图第一步)。key={lo.id,hi.id};块以 self=lo 朝向存储。
    // gen_lo/gen_hi 记录算定时两车的 path_gen;任一方 path_gen 变(换了固定路径)即失效
    // 重算。把"逐拍沿固定路径全程精扫"摊销成"每对路径只算一次",冲突几何恒定可见、不抖动。
    struct ConflictCacheEntry {
        int gen_lo = -1;
        int gen_hi = -1;
        std::vector<ConflictZone> blocks;  // canonical: s_self_*=lo, s_other_*=hi
        // Lazily reconstructed RViz geometry, aligned one-to-one with blocks.
        // It is diagnostic-only and has no role in conflict decisions.
        std::vector<std::vector<std::vector<ConflictMarker::Point>>>
            display_overlap_polygons;
    };
    mutable std::map<std::pair<int, int>, ConflictCacheEntry> conflict_cache_;
    mutable std::map<std::pair<int, int>, ConflictCacheEntry>
        future_a1_conflict_cache_;

    FutureA1Commitment future_a1_commitment_;
    std::set<std::pair<int, int>> future_a1_admission_logged_;
    int future_a1_admission_log_owner_id_ = -1;
    int future_a1_admission_log_owner_path_gen_ = -1;

    // Phase 2 资源模型:资源地图(只读)+ 资源令牌表(跨周期持久,§11.10)。
    const TrafficResourceMap* resmap_ = nullptr;
    ResourceTokenTable tokens_;
    std::function<void(const std::string&)> coord_log_sink_;
    std::string debug_log_source_ = "REAL";
    uint64_t debug_log_plan_id_ = 0;
    int debug_log_frame_id_ = -1;
    int debug_log_rollout_step_ = -1;
    // Intentionally excluded from SimSnapshot: rollout evaluations are
    // diagnostic evidence, and these counters never affect restored control.
    DynamicSpeedMetrics dynamic_speed_metrics_;
    double now_ = 0.0;  // 内部仿真时钟(每 decide 累加 dt),供令牌防抖/超时用
};

}  // namespace multi_vehicle
}  // namespace forklift_planner
