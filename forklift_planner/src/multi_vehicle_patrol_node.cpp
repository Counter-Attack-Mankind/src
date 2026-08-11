#include <ros/ros.h>
#include <ros/package.h>

#include <algorithm>
#include <array>
#include <clocale>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <deque>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "forklift_map/forklift_map.h"
#include "forklift_map/map_param.h"
#include "forklift_planner/multi_vehicle/cluster_admission_counterfactual_simulator.h"
#include "forklift_planner/multi_vehicle/dynamic_cluster_admission_shadow.h"
#include "forklift_planner/multi_vehicle/footprint.h"
#include "forklift_planner/multi_vehicle/future_a1_policy.h"
#include "forklift_planner/multi_vehicle/future_cluster_admission_shadow.h"
#include "forklift_planner/multi_vehicle/future_cluster_arbitration_shadow.h"
#include "forklift_planner/multi_vehicle/future_conflict_cluster_shadow.h"
#include "forklift_planner/multi_vehicle/future_conflict_zone_shadow.h"
#include "forklift_planner/multi_vehicle/future_mission_trajectory.h"
#include "forklift_planner/multi_vehicle/marker_publisher.h"
#include "forklift_planner/multi_vehicle/multi_vehicle_config.h"
#include "forklift_planner/multi_vehicle/prediction_shadow_comparator.h"
#include "forklift_planner/multi_vehicle/rule_engine.h"
#include "forklift_planner/multi_vehicle/task_allocator.h"
#include "forklift_planner/multi_vehicle/timed_conflict_shadow_checker.h"
#include "forklift_planner/multi_vehicle/traffic_resource_map.h"
#include "forklift_planner/path_generator.h"
#include "forklift_planner/planner_param.h"
#include "geometry_msgs/Point.h"
#include "sandbox_msgs/AprilObject.h"
#include "sandbox_msgs/Trajectory.h"
#include "sandbox_msgs/TrajectoryPoint.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

namespace {

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

}  // namespace

class MultiVehiclePatrolNode {
public:
    MultiVehiclePatrolNode() : nh_("~") {
        ros::NodeHandle param_nh;
        mp_ = MapParam::fromROSParam(param_nh);
        pp_ = PlannerParam::fromROSParam(param_nh);
        cfg_ = forklift_planner::multi_vehicle::MultiVehicleConfig::fromROSParam(
            param_nh);
        nh_.param("target_only", target_only_, -1);
        nh_.param("one_shot", one_shot_, one_shot_);
        const std::string planner_package =
            ros::package::getPath("forklift_planner");
        const std::string default_log_dir = planner_package.empty()
            ? "forklift_planner/logs" : planner_package + "/logs";
        nh_.param<std::string>("debug_log_dir", debug_log_dir_,
                               default_log_dir);
        nh_.param<std::string>("coord_log_file", coord_log_file_,
                               debug_log_dir_ +
                                   "/multi_vehicle_coordination.log");
        onset_log_file_ = debug_log_dir_ + "/forklift_onset.log";
        realbridge_positions_file_ =
            debug_log_dir_ + "/realbridge_positions.txt";
        initCoordLog();
        rb_horizon_ = cfg_.rolling_horizon;
        rb_horizon_refresh_period_ = cfg_.rolling_refresh_period;
        rb_horizon_refresh_ = std::max(
            1, static_cast<int>(std::lround(rb_horizon_refresh_period_ * pp_.update_rate)));
        rb_one_shot_traj_ = cfg_.one_shot_traj;
        if (cfg_.use_a1_cycle && rb_one_shot_traj_) {
            ROS_WARN("[multi_patrol][A1] one_shot_traj is incompatible with "
                     "task assignment at A1; forcing rolling-horizon mode");
            cfg_.one_shot_traj = false;
            rb_one_shot_traj_ = false;
        }
        if (target_only_ >= 0) {
            target_only_ = std::max(0, std::min(7, target_only_));
            cfg_.vehicle_count = std::max(cfg_.vehicle_count, target_only_ + 1);
            ROS_WARN("[real] single-target mode: only V%d will receive a task/controller; "
                     "its path still uses start_slots[%d] -> target_slots[%d].",
                     target_only_, target_only_, target_only_);
        }

        map_ = std::make_unique<ForkliftMap>(mp_);
        resource_map_ = std::make_unique<
            forklift_planner::multi_vehicle::TrafficResourceMap>(
            mp_, map_->slots(), map_->road_segments());
        generator_ = std::make_unique<PathGenerator>(mp_, pp_);
        allocator_ = std::make_unique<forklift_planner::multi_vehicle::TaskAllocator>(
            mp_, pp_, cfg_, *map_, *generator_);
        rule_engine_ = std::make_unique<forklift_planner::multi_vehicle::RuleEngine>(
            mp_, cfg_);
        future_mission_plan_builder_ = std::make_unique<
            forklift_planner::multi_vehicle::FutureMissionPlanBuilder>(cfg_);
        future_trajectory_generator_ = std::make_unique<
            forklift_planner::multi_vehicle::FutureTrajectoryGenerator>(mp_, cfg_);
        legacy_prediction_shadow_generator_ = std::make_unique<
            forklift_planner::multi_vehicle::LegacyPredictionShadowGenerator>(
                mp_, cfg_);
        prediction_shadow_comparator_ = std::make_unique<
            forklift_planner::multi_vehicle::PredictionShadowComparator>();
        timed_conflict_shadow_checker_ = std::make_unique<
            forklift_planner::multi_vehicle::TimedConflictShadowChecker>(
                mp_, cfg_);
        future_conflict_zone_shadow_builder_ = std::make_unique<
            forklift_planner::multi_vehicle::
                FutureConflictZoneShadowBuilder>(mp_, cfg_);
        future_conflict_cluster_shadow_builder_ = std::make_unique<
            forklift_planner::multi_vehicle::
                FutureConflictClusterShadowBuilder>();
        future_cluster_arbitration_shadow_ = std::make_unique<
            forklift_planner::multi_vehicle::
                FutureClusterArbitrationShadow>();
        future_cluster_admission_shadow_ = std::make_unique<
            forklift_planner::multi_vehicle::
                FutureClusterAdmissionShadow>();
        future_cluster_admission_shadow_tracker_ = std::make_unique<
            forklift_planner::multi_vehicle::
                FutureClusterAdmissionShadowTracker>();
        cluster_admission_evaluator_ = std::make_unique<
            forklift_planner::multi_vehicle::ClusterAdmissionEvaluator>();
        cluster_admission_counterfactual_simulator_ = std::make_unique<
            forklift_planner::multi_vehicle::
                ClusterAdmissionCounterfactualSimulator>();
        const auto coord_log_sink = [this](const std::string& line) {
            coordLog(line);
        };
        allocator_->setCoordLogSink(coord_log_sink);
        rule_engine_->setCoordLogSink(coord_log_sink);
        setCoordLogContext("REAL", 0, -1, -1);
        rule_engine_->setResourceMap(resource_map_.get());
        // A方案:仿真也画每车完整轨迹。real 模式 setupRealIO 会再 advertise(同topic,无害)。
        horizon_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);

        if (cfg_.precompute_task_filter) {
            allocator_->buildCache();
        }
        const Slot a1_pickup = allocator_->a1PickupSlot();
        marker_pub_ = std::make_unique<forklift_planner::multi_vehicle::MarkerPublisher>(
            nh_, mp_, pp_, map_->slots(), cfg_, a1_pickup);
        initAgents();
        visited_slots_.assign(map_->slots().size(), false);
        one_shot_done_.assign(agents_.size(), false);
        dumpResourceSpans();  // Phase 1.3 验证:打印各车路径经过的资源占用区间

        if (cfg_.real_mode) {
            setupRealIO();  // 实车模式:建 /object 订阅 + /traj_i//coord_speed_i 发布,打印摆位
            timer_ = nh_.createTimer(ros::Duration(1.0 / pp_.update_rate),
                                     &MultiVehiclePatrolNode::tick, this);
            ROS_WARN("[multi_patrol] *** 实车模式 real_mode=true ***:位置取 /object,"
                     "发 /traj_i + /coord_speed_i;协调与 sim 一致。");
            return;  // 实车模式不走 batch
        }

        // 1. 无头批处理快速回归:~batch_ticks>0 时不建实时 timer,由 main 调 runBatch 狂跑。
        // 也支持 ~batch_minutes(按仿真分钟换算成拍数,更直观)。
        ros::NodeHandle pnh("~");
        int batch_ticks = 0;
        double batch_minutes = 0.0;
        pnh.param("batch_ticks", batch_ticks, 0);
        pnh.param("batch_minutes", batch_minutes, 0.0);
        pnh.param("cluster_counterfactual_shadow",
                  cluster_counterfactual_shadow_enabled_, false);
        if (batch_ticks <= 0 && batch_minutes > 0.0)
            batch_ticks = static_cast<int>(batch_minutes * 60.0 * pp_.update_rate);
        cfg_batch_ticks_ = batch_ticks > 0 ? static_cast<unsigned long long>(batch_ticks) : 0;
        if (cluster_counterfactual_shadow_enabled_) {
            ROS_WARN("[PHASE63] counterfactual cluster admission simulation "
                     "enabled; this process is a shadow experiment and does "
                     "not represent baseline RuleEngine behavior");
        }
        if (cfg_batch_ticks_ > 0) {
            ROS_WARN("[batch] 无头快速回归模式:将狂跑 %llu 拍(≈%.0f 仿真分钟),"
                     "不发 marker、不按实时。", cfg_batch_ticks_,
                     cfg_batch_ticks_ / (pp_.update_rate * 60.0));
            return;  // 不建 timer
        }

        // 2. ROS实时模式，创建timer，按照步长调用tick()函数

        timer_ = nh_.createTimer(ros::Duration(1.0 / pp_.update_rate),
                                 &MultiVehiclePatrolNode::tick, this);

        ROS_INFO("[multi_patrol] started RViz timestamp simulation: vehicles=%d "
                 "seed=%d speed=%.2f max=%.2f dwell=%.2f horizon=%.2f step=%.2f",
                 cfg_.vehicle_count, cfg_.random_seed, cfg_.nominal_speed,
                 cfg_.max_speed, cfg_.dwell_time, cfg_.prediction_horizon,
                 cfg_.prediction_step);
    }

private:
    using VehicleAgent = forklift_planner::multi_vehicle::VehicleAgent;
    using VehicleAction = forklift_planner::multi_vehicle::VehicleAction;
    using VehicleMode = forklift_planner::multi_vehicle::VehicleMode;
    using MissionPhase = forklift_planner::multi_vehicle::MissionPhase;
    using LegTargetKind = forklift_planner::multi_vehicle::LegTargetKind;
    using FutureCertainty =
        forklift_planner::multi_vehicle::FutureCertainty;
    using FutureMissionTrajectory =
        forklift_planner::multi_vehicle::FutureMissionTrajectory;
    using FutureSegmentType =
        forklift_planner::multi_vehicle::FutureSegmentType;
    using PredictionShadowReport =
        forklift_planner::multi_vehicle::PredictionShadowReport;
    using TimedConflictShadowReport =
        forklift_planner::multi_vehicle::TimedConflictShadowReport;
    using FutureConflictZone =
        forklift_planner::multi_vehicle::FutureConflictZone;
    using FutureConflictCluster =
        forklift_planner::multi_vehicle::FutureConflictCluster;
    using ClusterReservationShadow =
        forklift_planner::multi_vehicle::ClusterReservationShadow;
    using ClusterAdmissionShadow =
        forklift_planner::multi_vehicle::ClusterAdmissionShadow;
    using ClusterAdmissionConstraint =
        forklift_planner::multi_vehicle::ClusterAdmissionConstraint;
    using ShadowVehicleState =
        forklift_planner::multi_vehicle::ShadowVehicleState;
    using CounterfactualShadowAction =
        forklift_planner::multi_vehicle::CounterfactualShadowAction;

    // Transitional simulation plan (stage 2 of the horizon refactor).
    // The planner still uses the existing sandbox world model for now, but
    // real simulation ticks execute these frozen decisions for one 2 s
    // commitment window instead of invoking ordinary arbitration again.
    struct SimPlannedAgentDecision {
        int path_gen = -1;
        VehicleMode mode = VehicleMode::NEED_TASK;
        VehicleAction action = VehicleAction::STOP;
        VehicleAction requested_action = VehicleAction::STOP;
        int blocker_id = -1;
        double wait_time = 0.0;
        double action_hold_remaining = 0.0;
        double cycle_break_immunity = 0.0;
        bool deadlock_breaker = false;
        double deadlock_breaker_hold = 0.0;
        std::string reason;
    };

    struct SimPlanFrame {
        std::vector<SimPlannedAgentDecision> agents;
        forklift_planner::multi_vehicle::RuleEngine::SimSnapshot rule_state;
    };

    struct A1ArrivalPrediction {
        int vehicle_id = -1;
        int path_gen = -1;
        double arrival_time = -1.0;
        double to_b_time = -1.0;
    };

    struct A1ArrivalSummary {
        std::map<int, A1ArrivalPrediction> candidates;
        std::map<int, std::string> excluded;
    };

    void initCoordLog() {
        std::error_code error;
        std::filesystem::create_directories(debug_log_dir_, error);
        if (error) {
            ROS_WARN("[multi_patrol] failed to create debug log directory %s: %s",
                     debug_log_dir_.c_str(), error.message().c_str());
            return;
        }
        const std::filesystem::path log_path(coord_log_file_);
        if (log_path.has_parent_path()) {
            std::filesystem::create_directories(log_path.parent_path(), error);
        }
        if (error) {
            ROS_WARN("[multi_patrol] failed to create log directory %s: %s",
                     log_path.parent_path().string().c_str(),
                     error.message().c_str());
            return;
        }
        coord_log_.open(coord_log_file_, std::ios::out | std::ios::trunc);
        if (!coord_log_) {
            ROS_WARN("[multi_patrol] failed to open coordination log: %s",
                     coord_log_file_.c_str());
            return;
        }
        coord_log_ << "[multi_patrol] coordination log started\n";
        coord_log_ << "vehicle_count=" << cfg_.vehicle_count
                   << " one_shot=" << (one_shot_ ? 1 : 0)
                   << " use_a1_cycle=" << (cfg_.use_a1_cycle ? 1 : 0)
                   << "\n";
        coord_log_.flush();
        ROS_WARN("[multi_patrol] coordination log: %s",
                 coord_log_file_.c_str());
    }

    void coordLog(const std::string& line) {
        coordLogWithContext(line, coord_log_source_, coord_log_plan_id_,
                            coord_log_frame_id_, coord_log_rollout_step_);
    }

    std::string contextualLog(const std::string& line,
                              const std::string& source,
                              uint64_t plan_id, int frame_id,
                              int rollout_step) const {
        char prefix[160];
        std::snprintf(prefix, sizeof(prefix),
                      "[SOURCE=%s] [plan=%llu] [frame=%d] "
                      "[rollout_step=%d] ",
                      source.c_str(),
                      static_cast<unsigned long long>(plan_id), frame_id,
                      rollout_step);
        return std::string(prefix) + line;
    }

    void coordLogWithContext(const std::string& line,
                             const std::string& source,
                             uint64_t plan_id, int frame_id,
                             int rollout_step) {
        if (!coord_log_ || coord_log_suppressed_) return;
        coord_log_ << contextualLog(line, source, plan_id, frame_id,
                                    rollout_step)
                   << "\n";
        coord_log_.flush();
    }

    void setCoordLogContext(const std::string& source, uint64_t plan_id,
                            int frame_id, int rollout_step) {
        coord_log_source_ = source;
        coord_log_plan_id_ = plan_id;
        coord_log_frame_id_ = frame_id;
        coord_log_rollout_step_ = rollout_step;
        if (rule_engine_) {
            rule_engine_->setDebugLogContext(source, plan_id, frame_id,
                                             rollout_step);
        }
    }

    std::string readableSimTime(double seconds) const {
        const double nonnegative = std::max(0.0, seconds);
        const long long tenths =
            static_cast<long long>(std::llround(nonnegative * 10.0));
        const long long minutes = tenths / 600;
        const double remainder = static_cast<double>(tenths % 600) / 10.0;
        char text[80];
        std::snprintf(text, sizeof(text), "%lldmin%.1fs", minutes, remainder);
        return text;
    }

    bool targetEnabled(int id) const {
        return target_only_ < 0 || id == target_only_;
    }

    const char* modeName(VehicleMode mode) const {
        switch (mode) {
            case VehicleMode::NEED_TASK: return "NEED_TASK";
            case VehicleMode::ACTIVE: return "ACTIVE";
            case VehicleMode::DWELL: return "DWELL";
        }
        return "UNKNOWN";
    }

    const char* missionPhaseName(MissionPhase phase) const {
        switch (phase) {
            case MissionPhase::DIRECT_TO_B: return "DIRECT_TO_B";
            case MissionPhase::TO_A1: return "TO_A1";
            case MissionPhase::PICKUP_DWELL: return "PICKUP_DWELL";
            case MissionPhase::WAIT_DROPOFF_TASK: return "WAIT_DROPOFF_TASK";
            case MissionPhase::TO_B: return "TO_B";
            case MissionPhase::UNLOAD_DWELL: return "UNLOAD_DWELL";
        }
        return "UNKNOWN";
    }

    const char* legTargetName(LegTargetKind target) const {
        switch (target) {
            case LegTargetKind::B_SLOT: return "B_SLOT";
            case LegTargetKind::A1: return "A1";
        }
        return "UNKNOWN";
    }

    void initAgents() {
        agents_.clear();
        agents_.reserve(static_cast<size_t>(cfg_.vehicle_count));

        const std::array<std_msgs::ColorRGBA, 8> colors = {
            rgba(0.00f, 0.28f, 0.82f),
            rgba(0.78f, 0.04f, 0.04f),
            rgba(0.00f, 0.52f, 0.16f),
            rgba(0.78f, 0.48f, 0.00f),
            rgba(0.46f, 0.16f, 0.78f),
            rgba(0.00f, 0.52f, 0.58f),
            rgba(0.82f, 0.24f, 0.00f),
            rgba(0.76f, 0.00f, 0.46f),
        };

        std::mt19937 rng(static_cast<unsigned int>(cfg_.random_seed));
        const int slot_count = static_cast<int>(map_->slots().size());

        // A1-cycle 的起点必须存在有效 B->A1 航段；普通模式要求能出库，
        // simple_forward_demo 还要求至少存在一个全程前进目标。
        auto startOK = [&](int s) {
            if (cfg_.use_a1_cycle) return allocator_->hasValidPickupLeg(s);
            if (!allocator_->hasValidOutbound(s)) return false;
            if (cfg_.simple_forward_demo && !allocator_->hasForwardTarget(s)) return false;
            return true;
        };

        // 简单测试版一次性诊断:贪心算 want 对互不冲突(起点≠终点、起点两两不同、终点两两不同)的
        // 无尖点前进对,分别按「最短(走得近)」和「最长(走得远)」各算一套,打印供离线预设到 start/target_slots。
        if (cfg_.simple_forward_demo) {
            const int want = std::max(1, cfg_.vehicle_count);
            auto greedyMatch = [&](bool prefer_long) {
                std::vector<int> cs, ct; std::vector<double> cl;
                std::vector<bool> us(slot_count, false), ut(slot_count, false);
                for (int iter = 0; iter < want; ++iter) {
                    int bs = -1, bt = -1; double blen = prefer_long ? -1.0 : 1e18;
                    for (int s = 0; s < slot_count; ++s) {
                        if (us[s] || ut[s] || !startOK(s)) continue;
                        std::vector<int> ft; std::vector<double> fl;
                        allocator_->forwardTargets(s, ft, &fl);  // 实际路径弧长
                        for (size_t k = 0; k < ft.size(); ++k) {
                            const int t = ft[k];
                            if (us[t] || ut[t] || t == s) continue;
                            const bool better = prefer_long ? (fl[k] > blen) : (fl[k] < blen);
                            if (better) { blen = fl[k]; bs = s; bt = t; }
                        }
                    }
                    if (bs < 0) break;
                    us[bs] = ut[bt] = true; cs.push_back(bs); ct.push_back(bt); cl.push_back(blen);
                }
                std::string ss, ts; double tot = 0.0;
                for (size_t k = 0; k < cs.size(); ++k) {
                    ss += std::to_string(cs[k]) + (k+1<cs.size()?", ":"");
                    ts += std::to_string(ct[k]) + (k+1<ct.size()?", ":"");
                    tot += cl[k];
                }
                ROS_WARN("[simple-scan] %s %zu 对(总长%.2fm):", prefer_long?"【走得远】":"【走得近】", cs.size(), tot);
                for (size_t k = 0; k < cs.size(); ++k)
                    ROS_WARN("[simple-scan]   对%zu: %d -> %d  len=%.3f", k+1, cs[k], ct[k], cl[k]);
                ROS_WARN("[simple-scan]   start_slots:  [%s]", ss.c_str());
                ROS_WARN("[simple-scan]   target_slots: [%s]", ts.c_str());
            };
            greedyMatch(false);  // 短
            greedyMatch(true);   // 长
        }

        std::vector<int> random_starts;
        if (cfg_.randomize_start) {
            for (int s = 0; s < slot_count; ++s) {
                if (startOK(s)) random_starts.push_back(s);
            }
            std::shuffle(random_starts.begin(), random_starts.end(), rng);
        }

        std::vector<bool> used(static_cast<size_t>(slot_count), false);
        for (int i = 0; i < cfg_.vehicle_count; ++i) {
            VehicleAgent v;
            v.id = i;
            int start_slot;
            if (cfg_.randomize_start && !random_starts.empty()) {
                start_slot = random_starts[static_cast<size_t>(i) %
                                           random_starts.size()];
            } else {
                start_slot = cfg_.start_slots.empty()
                    ? i
                    : cfg_.start_slots[static_cast<size_t>(i) %
                                       cfg_.start_slots.size()];
            }
            start_slot = ((start_slot % slot_count) + slot_count) % slot_count;

            // 简单测试版下 startOK 还要求该起点有「全程前进(无尖点)」目标,保证每车都能一把开进。
            if (used[start_slot] || !startOK(start_slot)) {
                int repl = -1;
                for (int s = 0; s < slot_count; ++s) {
                    if (!used[s] && startOK(s)) {
                        repl = s;
                        break;
                    }
                }
                if (repl >= 0) {
                    if (!startOK(start_slot)) {
                        ROS_WARN("[multi_patrol] start slot %d 不可用"
                                 "(陷阱/无前进目标); V%d 改从 slot %d 起步",
                                 start_slot, i, repl);
                    }
                    start_slot = repl;
                }
            }
            used[start_slot] = true;

            v.current_slot = start_slot;
            v.target_slot = v.current_slot;
            if (cfg_.use_a1_cycle) {
                v.loaded = false;
                v.mission_phase = MissionPhase::TO_A1;
                v.leg_target = LegTargetKind::A1;
            } else {
                std::bernoulli_distribution load_dist(0.5);
                v.loaded = load_dist(rng);
                v.mission_phase = MissionPhase::DIRECT_TO_B;
                v.leg_target = LegTargetKind::B_SLOT;
            }
            v.color = colors[static_cast<size_t>(i) % colors.size()];
            v.mode = VehicleMode::NEED_TASK;
            agents_.push_back(v);
        }

        int enabled_count = 0;
        int assigned_count = 0;
        for (VehicleAgent& v : agents_) {
            if (targetEnabled(v.id)) {
                ++enabled_count;
                if (allocator_->assignNextTask(v, agents_)) {
                    ++assigned_count;
                }
            }
        }
        if (cfg_.use_a1_cycle) {
            ROS_INFO("[multi_patrol][A1] initial pickup legs assigned: %d/%d",
                     assigned_count, enabled_count);
            if (assigned_count != enabled_count) {
                ROS_ERROR("[multi_patrol][A1] %d enabled vehicle(s) have no "
                          "initial B->A1 task; inspect the preceding path errors",
                          enabled_count - assigned_count);
            }
        }
        force_horizon_refresh_ = true;
        resetStatusLogState();
    }

    // Phase 1.3 验证:对每辆 active 车,打印其固定路径经过的资源及弧长区间,
    // 用于核对资源地图(SLOT_BODY/DOCK 等)与实际路径一致。一次性。
    void dumpResourceSpans() {
        ROS_INFO("[res_map] resources built: %zu",
                 resource_map_->resources().size());
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) continue;
            const auto spans = resource_map_->spansForPath(v.track);
            for (const auto& sp : spans) {
                const auto* r = resource_map_->byId(sp.resource_id);
                ROS_INFO("[res_map] V%d uses %s s=[%.3f,%.3f] (len=%.3f)",
                         v.id, r ? r->name.c_str() : "?",
                         sp.s_enter, sp.s_exit, v.track.length());
            }
        }
    }

    void resetStatusLogState() {
        const size_t n = agents_.size();
        last_logged_mode_.assign(n, VehicleMode::NEED_TASK);
        last_logged_action_.assign(n, VehicleAction::STOP);
        last_logged_reason_.assign(n, "");
        last_logged_blocker_.assign(n, -999);
        last_logged_task_count_.assign(n, -1);
        last_logged_mission_phase_.assign(n, MissionPhase::DIRECT_TO_B);
        last_status_log_time_.assign(n, ros::Time(0));
        last_diag_time_.assign(n, ros::Time(0));
    }

    double limitedSpeed(double current, double desired, double dt) const {
        if (desired > current) {
            return std::min(desired, current + cfg_.max_accel * dt);
        }
        return std::max(desired, current - cfg_.max_decel * dt);
    }

    RoughWp poseForCollision(const VehicleAgent& v, double path_s) const {
        if (!v.track.empty()) {
            if (v.mode == VehicleMode::DWELL) {
                return v.track.poseAtS(v.track.length());
            }
            return v.track.poseAtS(std::min(path_s, v.track.length()));
        }
        // 无轨迹的 idle 车(NEED_TASK)仍物理停在 current_slot 上，别的车不能从它身上
        // 碾过去。车身中心在 dock，朝向朝库外(pre_dock 方向)，参考点沿鼻向后移 d。
        const Slot& s = map_->slots().at(static_cast<size_t>(v.current_slot));
        const double th = std::atan2(s.pre_dock_y - s.dock_y(),
                                     s.pre_dock_x - s.dock_x());
        RoughWp p;
        p.x = s.dock_x() - mp_.rear_axle_to_center * std::cos(th);
        p.y = s.dock_y() - mp_.rear_axle_to_center * std::sin(th);
        p.theta = th;
        p.type = WpType::FORWARD;
        return p;
    }

    // 前瞻预测(集中式全信息+确定性):克隆全局状态,复用真实 updateDwellAndTasks+decide+
    // advanceVehicles 闭环空跑 H 拍(预测检查被 sim_mode_ 屏蔽,不递归),若 H 内出现「持续闭环
    // 死锁」(findDeadlockMembers 非空,wait≥kWait 的全停闭环)即返回 true。跑完精确还原,对真实零影响。
    bool simPredictsDeadlock() {
        const double dt = 1.0 / pp_.update_rate;
        constexpr int H = 400;          // 前瞻 ~40s
        constexpr double kWait = 10.0;  // 全停闭环持续此秒数 = 真死锁
        const std::vector<VehicleAgent> sa = agents_;
        const std::vector<bool> sv = visited_slots_;
        const std::string previous_log_source = coord_log_source_;
        const uint64_t previous_log_plan = coord_log_plan_id_;
        const int previous_log_frame = coord_log_frame_id_;
        const int previous_rollout_step = coord_log_rollout_step_;
        const uint64_t rollout_plan_id = ++rollout_log_id_;
        setCoordLogContext("ROLLOUT", rollout_plan_id, 0, 0);
        const auto sr = rule_engine_->snapshot();
        const auto sl = allocator_->snapshot();
        const bool prev = sim_mode_;
        sim_mode_ = true;
        bool dead = false;
        for (int s = 0; s < H && !dead; ++s) {
            setCoordLogContext("ROLLOUT", rollout_plan_id, s, s + 1);
            updateDwellAndTasks(dt);
            rule_engine_->decide(agents_, dt);
            advanceVehicles(dt);
            if (!findDeadlockMembers(kWait).empty()) dead = true;
        }
        agents_ = sa;
        visited_slots_ = sv;
        coord_log_suppressed_ = true;
        rule_engine_->restore(sr);
        allocator_->restore(sl);
        coord_log_suppressed_ = false;
        sim_mode_ = prev;
        setCoordLogContext(previous_log_source, previous_log_plan,
                           previous_log_frame, previous_rollout_step);
        return dead;
    }

    // ───────── AD 滚动时域:世界模型推演 → 有限时域时间参数化轨迹 ─────────
    // 零自由度=全局确定性:从当前真实状态(/object 位置 + 动捕差分速度,已在 agents_ 里)出发,
    // 复用真实 updateDwellAndTasks+decide+advanceVehicles 精确前推 horizon 秒,逐拍记录每车
    // (x,y,yaw,带符号速度,time)。这条轨迹运动学完备(模型自带曲率限速/加减速/停止/倒车换向过零),
    // 位置序列隐含方向→控制器纯跟踪即可,不需 coord_speed 符号/typeAtS 那套。每周期刷新=滚动时域。
    // hold[i]=true 表示该车整段不动(静止特例)→ 控制器 idle 不控制。

    void rollWorldModel(double horizon, std::vector<sandbox_msgs::Trajectory>& out,
                        std::vector<bool>& hold,
                        std::vector<SimPlanFrame>* plan_frames = nullptr,
                        bool first_step_task_state_is_current = false) {

        const double dt = 1.0 / pp_.update_rate;            //系统每触发一次，就向前推进dt时间
        const int H = std::max(1, (int)std::lround(horizon / dt));      //四舍五入决定仿真步数，但至少模拟1步

        //=====（初始化轨迹，全部置空并且默认车辆均为静止状态）===========
        const size_t n = agents_.size();
        out.assign(n, sandbox_msgs::Trajectory{});
        hold.assign(n, true);
        const std::string previous_log_source = coord_log_source_;
        const uint64_t previous_log_plan = coord_log_plan_id_;
        const int previous_log_frame = coord_log_frame_id_;
        const int previous_rollout_step = coord_log_rollout_step_;
        const uint64_t rollout_plan_id =
            plan_frames != nullptr ? sim_plan_id_ + 1 : ++rollout_log_id_;
        setCoordLogContext("ROLLOUT", rollout_plan_id, 0, 0);
        //初始化轨迹，所有离散点中的目标点按序排列，并且坐标系设为世界坐标系
        for (size_t i = 0; i < n; ++i) { out[i].target = (int)i; out[i].header.frame_id = "world"; }

        //===========（状态快照与回滚）============
        const std::vector<VehicleAgent> sa = agents_;    //（将Agents通过拷贝构造函数给sa，sa设为const，后续仅改变备份的agents，对显示不产生影响）  
        const std::vector<bool> sv = visited_slots_;
        const auto sr = rule_engine_->snapshot();       //保存规则引擎状态
        const auto sl = allocator_->snapshot();         //保存任务分配器状态
        const bool prev = sim_mode_;                    //保存现在模式（仿真或是实际）
        sim_mode_ = true;       //切换到仿真模式，因为现在属于提前规划，必须视为仿真

        //=========（创建匿名函数recored，用来记录当前车辆状态，并每拍给每辆车生成一个TrajPoint）========
        auto record = [&](int s) {
            for (size_t i = 0; i < n; ++i) {
                const VehicleAgent& v = agents_[i];         //对每辆车都记录一次当前状态，v是agents_[i] 的只读引用
                const RoughWp p = poseForCollision(v, v.path_s);    //根据车辆v当前走到路径上的距离 path_s，求它当前在地图里的姿态
                // 几何判前进/倒车(用户判据):存的航向(=车头)与路径切向(=前进方向)反向→倒车。
                // 不靠 typeAtS 标签(可能没标对)。前进段速度取正、倒车段取负——就这么简单。
                
                //====判断是否倒车=====
                bool rev = false;
                if (!v.track.empty()) {
                    const double L = v.track.length(), ds = 0.05;
                    const double s = std::min(std::max(v.path_s, 0.0), L);
                    const auto pa = v.track.poseAtS(std::max(0.0, s - ds));
                    const auto pc = v.track.poseAtS(std::min(L, s + ds));
                    rev = std::cos(p.theta - std::atan2(pc.y - pa.y, pc.x - pa.x)) < 0.0;
                }
                sandbox_msgs::TrajectoryPoint tp;
                tp.x = p.x; tp.y = p.y; tp.yaw = p.theta;                          // 车头朝向
                tp.velocity = (rev ? -1.0 : 1.0) * std::max(0.0, v.current_speed); // 前进+/倒车-
                tp.time = s * dt;
                out[i].points.push_back(tp);
                if (v.current_speed > 1e-3) hold[i] = false;   // 整段都不动才算 hold
            }
        };

        //=====（将刚才判断记录的，作为预测的第0个点）====
        record(0);
        if (plan_frames != nullptr) {
            plan_frames->clear();
            plan_frames->reserve(static_cast<size_t>(H));
        }

        //开始向未来预测H步长，H = 预测时间/dt
        for (int s = 1; s <= H; ++s) {
            setCoordLogContext("ROLLOUT", rollout_plan_id, s - 1, s);
            // The simulation executor calls updateDwellAndTasks() before it
            // asks for a new plan. Its first planned control therefore starts
            // from that already-updated task state; later frames advance the
            // task state normally. Existing real rollout keeps the old order.
            if (!(first_step_task_state_is_current && s == 1)) {
                updateDwellAndTasks(dt);
            }
            if (plan_frames != nullptr) {
                // All decisions used to build this active plan share one
                // absolute end time. At future offset tau, only [tau, H]
                // remains visible; never open a fresh H-second window and
                // accidentally reason out to 2H.
                const double tau = static_cast<double>(s - 1) * dt;
                const double remaining_horizon =
                    std::max(dt, horizon - tau);
                rule_engine_->decide(agents_, dt, remaining_horizon);
            } else {
                rule_engine_->decide(agents_, dt);
            }
            if (plan_frames != nullptr) {
                SimPlanFrame frame;
                frame.agents.reserve(agents_.size());
                for (const VehicleAgent& v : agents_) {
                    SimPlannedAgentDecision d;
                    d.path_gen = v.path_gen;
                    d.mode = v.mode;
                    d.action = v.action;
                    d.requested_action = v.requested_action;
                    d.blocker_id = v.blocker_id;
                    d.wait_time = v.wait_time;
                    d.action_hold_remaining = v.action_hold_remaining;
                    d.cycle_break_immunity = v.cycle_break_immunity;
                    d.deadlock_breaker = v.deadlock_breaker;
                    d.deadlock_breaker_hold = v.deadlock_breaker_hold;
                    d.reason = v.reason;
                    frame.agents.push_back(std::move(d));
                }
                frame.rule_state = rule_engine_->snapshot();
                plan_frames->push_back(std::move(frame));
            }
            advanceVehicles(dt);
            record(s);
        }

        //将沙盒预测造成所有的改动恢复
        agents_ = sa;
        visited_slots_ = sv;
        coord_log_suppressed_ = true;
        rule_engine_->restore(sr);
        allocator_->restore(sl);
        coord_log_suppressed_ = false;
        sim_mode_ = prev;
        setCoordLogContext(previous_log_source, previous_log_plan,
                           previous_log_frame, previous_rollout_step);
    }

    // Predict A1 service arrival from each vehicle's current B->A1 leg in
    // isolation. Coordination actions are intentionally excluded here: the
    // owner must be chosen before ordinary pairwise rules can stop a candidate.
    A1ArrivalSummary predictA1Arrivals(double horizon) const {
        A1ArrivalSummary summary;
        const double dt = 1.0 / pp_.update_rate;
        const int max_steps = std::max(
            0, static_cast<int>(std::ceil(std::max(0.0, horizon) / dt)));

        for (const VehicleAgent& v : agents_) {
            if (!targetEnabled(v.id) || !v.active() ||
                v.mission_phase != MissionPhase::TO_A1 ||
                v.leg_target != LegTargetKind::A1 || v.track.empty()) {
                continue;
            }

            VehicleAgent preview = v;
            double arrival_time = -1.0;
            if (preview.path_s >= preview.track.length() - 1e-9) {
                arrival_time = 0.0;
            } else {
                for (int step = 1;
                     step <= max_steps &&
                     preview.path_s < preview.track.length() - 1e-9;
                     ++step) {
                    const double desired = std::min(
                        rule_engine_->speedForAction(VehicleAction::NOMINAL),
                        curvatureSpeed(preview));
                    if (!std::isfinite(desired) || desired <= 1e-9) break;
                    preview.current_speed = limitedSpeed(
                        preview.current_speed, desired, dt);
                    const double old_s = preview.path_s;
                    preview.path_s = std::min(
                        preview.track.length(),
                        preview.path_s + preview.current_speed * dt);
                    if (preview.path_s <= old_s + 1e-12) break;
                    if (preview.path_s >= preview.track.length() - 1e-9) {
                        arrival_time = static_cast<double>(step) * dt;
                    }
                }
            }
            if (!forklift_planner::multi_vehicle::
                    futureA1ArrivalWithinHorizon(arrival_time, horizon)) {
                summary.excluded[v.id] = "horizon_exceeded";
                continue;
            }

            A1ArrivalPrediction prediction;
            prediction.vehicle_id = v.id;
            prediction.path_gen = v.path_gen;
            prediction.arrival_time = arrival_time;
            prediction.to_b_time = arrival_time + cfg_.pickup_dwell_time;
            summary.candidates[v.id] = prediction;
        }
        return summary;
    }

    forklift_planner::multi_vehicle::RuleEngine::FutureA1Commitment
    selectFutureA1Owner(const A1ArrivalSummary& summary) const {
        using Commitment = forklift_planner::multi_vehicle::RuleEngine::
            FutureA1Commitment;
        const double tie_window = std::max(0.02, cfg_.prediction_step);
        std::vector<forklift_planner::multi_vehicle::FutureA1RankedCandidate>
            ranked;
        ranked.reserve(summary.candidates.size());
        for (const auto& item : summary.candidates) {
            const A1ArrivalPrediction& candidate = item.second;
            ranked.push_back({candidate.vehicle_id, candidate.arrival_time});
        }

        const int best_id =
            forklift_planner::multi_vehicle::selectFutureA1Candidate(
                ranked, tie_window, [this](int lhs, int rhs) {
                    const VehicleAgent* lhs_agent = agentById_c(lhs);
                    const VehicleAgent* rhs_agent = agentById_c(rhs);
                    if (lhs_agent == nullptr || rhs_agent == nullptr) return -1;
                    return rule_engine_->unifiedPriority(*lhs_agent,
                                                         *rhs_agent);
                });

        Commitment commitment;
        const auto best = summary.candidates.find(best_id);
        if (best == summary.candidates.end()) return commitment;
        commitment.owner_id = best->second.vehicle_id;
        commitment.owner_path_gen = best->second.path_gen;
        commitment.predicted_a1_arrival_time = best->second.arrival_time;
        commitment.predicted_to_b_time = best->second.to_b_time;
        return commitment;
    }

    forklift_planner::multi_vehicle::RuleEngine::FutureA1Commitment
    retainLockedFutureA1Owner() const {
        using Commitment = forklift_planner::multi_vehicle::RuleEngine::
            FutureA1Commitment;
        if (!future_a1_commitment_.valid()) return Commitment{};

        const VehicleAgent* owner =
            agentById_c(future_a1_commitment_.owner_id);
        if (owner == nullptr ||
            owner->path_gen != future_a1_commitment_.owner_path_gen ||
            !owner->pending_dropoff_valid ||
            owner->pending_dropoff_track.empty()) {
            return Commitment{};
        }

        if (owner->mode != VehicleMode::DWELL ||
            owner->mission_phase != MissionPhase::PICKUP_DWELL) {
            return Commitment{};
        }
        Commitment retained = future_a1_commitment_;
        retained.predicted_a1_arrival_time = 0.0;
        retained.predicted_to_b_time = std::max(0.0, owner->dwell_remaining);
        return retained;
    }

    std::string futureA1ChangeReason(
        const forklift_planner::multi_vehicle::RuleEngine::
            FutureA1Commitment& previous,
        const forklift_planner::multi_vehicle::RuleEngine::
            FutureA1Commitment& current,
        const A1ArrivalSummary& summary,
        bool owner_locked) const {
        const VehicleAgent* old_owner = agentById_c(previous.owner_id);
        const bool departure_handoff =
            old_owner != nullptr &&
            old_owner->mission_phase == MissionPhase::TO_B;
        const bool previous_owner_valid =
            old_owner != nullptr &&
            old_owner->path_gen == previous.owner_path_gen &&
            old_owner->pending_dropoff_valid &&
            !old_owner->pending_dropoff_track.empty() &&
            (old_owner->mission_phase == MissionPhase::TO_A1 ||
             old_owner->mission_phase == MissionPhase::PICKUP_DWELL);
        const auto excluded = summary.excluded.find(previous.owner_id);
        const bool horizon_exceeded =
            excluded != summary.excluded.end() &&
            excluded->second == "horizon_exceeded";
        return forklift_planner::multi_vehicle::futureA1TransitionReason(
            previous.valid(), previous.owner_id,
            current.valid(), current.owner_id,
            owner_locked, previous_owner_valid, horizon_exceeded,
            departure_handoff);
    }

    void logFutureA1Transition(
        const forklift_planner::multi_vehicle::RuleEngine::
            FutureA1Commitment& previous,
        const forklift_planner::multi_vehicle::RuleEngine::
            FutureA1Commitment& current,
        const A1ArrivalSummary& summary,
        const std::string& change_reason) {
        const bool same_owner =
            previous.valid() && current.valid() &&
            previous.owner_id == current.owner_id &&
            previous.owner_path_gen == current.owner_path_gen;
        std::string event;
        if (same_owner) {
            event = "HOLD";
        } else if (!previous.valid()) {
            event = "CREATE";
        } else if (!current.valid()) {
            event = "RELEASE";
        } else if (previous.valid()) {
            event = "CHANGE";
        } else {
            event = "HOLD";
        }

        std::ostringstream candidates;
        candidates << "[";
        bool first = true;
        for (const auto& item : summary.candidates) {
            if (!first) candidates << ",";
            first = false;
            candidates << "V" << item.first << ":" << std::fixed
                       << std::setprecision(2) << item.second.arrival_time
                       << "s";
        }
        candidates << "]";
        std::ostringstream excluded;
        excluded << "[";
        first = true;
        for (const auto& item : summary.excluded) {
            if (!first) excluded << ",";
            first = false;
            excluded << "V" << item.first << ":" << item.second;
        }
        excluded << "]";

        char line[1024];
        if (current.valid()) {
            const std::string old_owner = previous.valid()
                ? "V" + std::to_string(previous.owner_id)
                : "none";
            std::snprintf(
                line, sizeof(line),
                "[FUTURE_A1] time=%s event=%s old=%s owner=V%d "
                "arrival=%.2fs to_b=%.2fs path_gen=%d "
                "horizon=%.2fs candidates=%s excluded=%s change_reason=%s",
                readableSimTime(sim_time_).c_str(), event.c_str(),
                old_owner.c_str(),
                current.owner_id, current.predicted_a1_arrival_time,
                current.predicted_to_b_time, current.owner_path_gen,
                rb_horizon_, candidates.str().c_str(),
                excluded.str().c_str(),
                change_reason.c_str());
        } else if (previous.valid()) {
            std::snprintf(
                line, sizeof(line),
                "[FUTURE_A1] time=%s event=RELEASE owner=V%d "
                "path_gen=%d horizon=%.2fs candidates=%s excluded=%s "
                "change_reason=%s",
                readableSimTime(sim_time_).c_str(), previous.owner_id,
                previous.owner_path_gen, rb_horizon_,
                candidates.str().c_str(), excluded.str().c_str(),
                change_reason.c_str());
        } else {
            std::snprintf(
                line, sizeof(line),
                "[FUTURE_A1] time=%s event=HOLD old=none owner=none "
                "horizon=%.2fs candidates=%s excluded=%s change_reason=%s",
                readableSimTime(sim_time_).c_str(), rb_horizon_,
                candidates.str().c_str(), excluded.str().c_str(),
                change_reason.c_str());
        }
        const std::string console_line = contextualLog(
            line, "ROLLOUT", sim_plan_id_, -1, -1);
        ROS_WARN("%s", console_line.c_str());
        coordLogWithContext(line, "ROLLOUT", sim_plan_id_, -1, -1);
    }

    void buildSimulationHorizonPlan(
        std::vector<sandbox_msgs::Trajectory>& trajs,
        std::vector<bool>& hold) {
        // Normal rolling simulation prepares previews in publishHorizon(),
        // while batch mode calls this function directly. Keep both paths
        // behaviorally identical.
        prepareA1DropoffPreviewsForHorizon();
        buildFutureMissionTrajectoryCache();

        const auto previous_commitment = future_a1_commitment_;
        const A1ArrivalSummary arrivals = predictA1Arrivals(rb_horizon_);
        auto commitment = retainLockedFutureA1Owner();
        const bool owner_locked = commitment.valid();
        if (!owner_locked) commitment = selectFutureA1Owner(arrivals);
        const std::string change_reason = futureA1ChangeReason(
            previous_commitment, commitment, arrivals, owner_locked);

        rule_engine_->clearFutureA1Commitment();
        if (commitment.valid()) {
            rule_engine_->setFutureA1Commitment(commitment);
        }
        std::vector<SimPlanFrame> frames;
        rollWorldModel(rb_horizon_, trajs, hold, &frames,
                       /*first_step_task_state_is_current=*/true);
        rule_engine_->clearFutureA1Commitment();
        future_a1_commitment_ = commitment;
        sim_plan_frames_ = std::move(frames);
        sim_plan_cursor_ = 0;
        sim_plan_valid_ = !sim_plan_frames_.empty();
        sim_plan_start_time_ = sim_time_;
        ++sim_plan_id_;
        evaluateFutureClusterArbitrationShadow();
        publishFutureClusterArbitrationShadowMarkers();
        publishFutureClusterAdmissionShadowMarkers();
        publishFutureClusterCounterfactualMarkers();
        logFutureA1Transition(previous_commitment, commitment, arrivals,
                              change_reason);
        ROS_INFO("[sim_plan] built plan=%llu start=%.2f horizon=%.2f "
                 "frames=%zu commit_frames=%d",
                 static_cast<unsigned long long>(sim_plan_id_),
                 sim_plan_start_time_, rb_horizon_, sim_plan_frames_.size(),
                 rb_horizon_refresh_);
        for (const VehicleAgent& v : agents_) {
            if (v.mode != VehicleMode::DWELL ||
                v.mission_phase != MissionPhase::PICKUP_DWELL ||
                !v.pending_dropoff_valid) {
                continue;
            }
            ROS_WARN("[multi_patrol][A1 EXIT HORIZON] V%d target=B%d "
                     "dwell_part=%.2fs departure_window=%.2fs "
                     "total_horizon=%.2fs",
                     v.id, v.pending_dropoff_slot,
                     std::min(v.dwell_remaining, rb_horizon_),
                     std::max(0.0, rb_horizon_ - v.dwell_remaining),
                     rb_horizon_);
        }
    }

    void buildFutureMissionTrajectoryCache() {
        future_trajectory_cache_.clear();
        future_trajectory_cache_.reserve(agents_.size());
        prediction_shadow_reports_.clear();
        prediction_shadow_reports_.reserve(agents_.size());
        legacy_prediction_cache_.clear();
        legacy_prediction_cache_.reserve(agents_.size());

        for (const VehicleAgent& vehicle : agents_) {
            std::optional<forklift_planner::multi_vehicle::PathTrack>
                next_pickup_track;
            int pickup_slot = -1;
            if (vehicle.mission_phase == MissionPhase::TO_B) {
                pickup_slot = vehicle.target_slot;
            } else if (vehicle.mission_phase == MissionPhase::UNLOAD_DWELL) {
                pickup_slot = vehicle.current_slot;
            }
            if (pickup_slot >= 0) {
                forklift_planner::multi_vehicle::PathTrack preview;
                if (allocator_->previewPickupTrack(pickup_slot, preview)) {
                    next_pickup_track = std::move(preview);
                }
            }

            FutureMissionTrajectory trajectory;
            trajectory.plan = future_mission_plan_builder_->build(
                vehicle, rb_horizon_, next_pickup_track);
            trajectory.samples =
                future_trajectory_generator_->generate(trajectory.plan);

            const auto legacy_samples =
                legacy_prediction_shadow_generator_->generate(vehicle,
                                                               rb_horizon_);
            const PredictionShadowReport shadow =
                prediction_shadow_comparator_->compare(
                    vehicle, legacy_samples, trajectory);
            prediction_shadow_reports_.push_back(shadow);
            legacy_prediction_cache_.push_back(legacy_samples);

            {
                std::ostringstream line;
                line << std::fixed << std::setprecision(2)
                     << "[FUTURE_SHADOW] vehicle=V" << vehicle.id
                     << " event=SUMMARY horizon=" << rb_horizon_
                     << " old_samples=" << shadow.old_sample_count
                     << " new_samples=" << shadow.new_sample_count
                     << " matched=" << shadow.matched_sample_count
                     << " segment_mismatch="
                     << shadow.segment_mismatch_count
                     << " time_mismatch=" << shadow.time_mismatch_count
                     << std::scientific << std::setprecision(6)
                     << " max_s_error=" << shadow.max_s_error
                     << " mean_s_error=" << shadow.mean_s_error
                     << " max_position_error="
                     << shadow.max_position_error
                     << " mean_position_error="
                     << shadow.mean_position_error
                     << " max_speed_error=" << shadow.max_speed_error
                     << " max_body_center_error="
                     << shadow.max_body_center_error
                     << " max_body_yaw_error="
                     << shadow.max_body_yaw_error;
                ROS_INFO("%s", line.str().c_str());
                coordLogWithContext(line.str(), "PREDICT", sim_plan_id_ + 1,
                                    -1, -1);
            }
            if (shadow.maximum_error.valid) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(6)
                     << "[FUTURE_SHADOW] vehicle=V" << vehicle.id
                     << " event=MAX_ERROR t=" << shadow.maximum_error.t
                     << " old_s=" << shadow.maximum_error.old_s
                     << " new_s=" << shadow.maximum_error.new_s
                     << " position_error="
                     << shadow.maximum_error.position_error
                     << " phase="
                     << missionPhaseName(shadow.maximum_error.new_phase)
                     << " segment="
                     << shadow.maximum_error.new_segment_id;
                ROS_INFO("%s", line.str().c_str());
                coordLogWithContext(line.str(), "PREDICT", sim_plan_id_ + 1,
                                    -1, -1);
            }
            if (shadow.first_mismatch.valid) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(2)
                     << "[FUTURE_SHADOW] vehicle=V" << vehicle.id
                     << " event=SEGMENT_MISMATCH t="
                     << shadow.first_mismatch.t
                     << " old="
                     << (shadow.first_mismatch.mismatch ==
                                 forklift_planner::multi_vehicle::
                                     ShadowMismatchKind::
                                         OLD_PREDICTION_UNAVAILABLE
                             ? "unavailable"
                             : "current_track")
                     << " new_phase="
                     << missionPhaseName(shadow.first_mismatch.new_phase)
                     << " new_segment="
                     << shadow.first_mismatch.new_segment_id
                     << " reason="
                     << forklift_planner::multi_vehicle::
                            shadowMismatchKindName(
                                shadow.first_mismatch.mismatch);
                ROS_INFO("%s", line.str().c_str());
                coordLogWithContext(line.str(), "PREDICT", sim_plan_id_ + 1,
                                    -1, -1);
            }

            for (const auto& segment : trajectory.plan.segments) {
                std::ostringstream line;
                line << std::fixed << std::setprecision(2)
                     << "[FUTURE_MISSION] vehicle=V" << vehicle.id
                     << " segment=" << segment.segment_id
                     << " phase=" << missionPhaseName(segment.phase)
                     << " type="
                     << forklift_planner::multi_vehicle::
                            futureSegmentTypeName(segment.type)
                     << " certainty="
                     << forklift_planner::multi_vehicle::
                            futureCertaintyName(segment.certainty)
                     << " start=" << segment.start_time
                     << " duration=" << segment.duration
                     << " path_gen="
                     << segment.mission_leg_id.expected_path_gen;
                ROS_INFO("%s", line.str().c_str());
                coordLogWithContext(line.str(), "PREDICT", sim_plan_id_ + 1,
                                    -1, -1);
            }
            future_trajectory_cache_.push_back(std::move(trajectory));
        }
        future_conflict_zones_ =
            future_conflict_zone_shadow_builder_->build(
                future_trajectory_cache_);
        future_conflict_clusters_ =
            future_conflict_cluster_shadow_builder_->build(
                future_conflict_zones_, sim_plan_id_ + 1);
        logFutureConflictZones();
        logFutureConflictClusters();
        runTimedConflictShadowComparison();
        publishFutureMissionMarkers();
        publishFutureShadowConflictMarkers();
        publishFutureConflictZoneShadowMarkers();
        publishFutureConflictClusterShadowMarkers();
    }

    void logFutureConflictZones() {
        for (const FutureConflictZone& zone : future_conflict_zones_) {
            if (zone.source != forklift_planner::multi_vehicle::
                                   FutureConflictZoneSource::FUTURE_SEGMENT) {
                continue;
            }
            const auto key = std::make_tuple(
                zone.vehicle_a, zone.vehicle_b, zone.segment_id_a,
                zone.segment_id_b, zone.path_generation_a,
                zone.path_generation_b,
                static_cast<long long>(std::llround(zone.s_a_enter * 1000.0)),
                static_cast<long long>(std::llround(zone.s_b_enter * 1000.0)));
            if (!future_conflict_zone_log_keys_.insert(key).second) continue;
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[FUTURE_CONFLICT_ZONE] pair=V" << zone.vehicle_a
                 << "-V" << zone.vehicle_b
                 << " future_zone=" << zone.future_zone_id
                 << " segment_a=" << zone.segment_id_a
                 << " phase_a=" << missionPhaseName(zone.phase_a)
                 << " certainty_a="
                 << forklift_planner::multi_vehicle::futureCertaintyName(
                        zone.certainty_a)
                 << " segment_b=" << zone.segment_id_b
                 << " phase_b=" << missionPhaseName(zone.phase_b)
                 << " certainty_b="
                 << forklift_planner::multi_vehicle::futureCertaintyName(
                        zone.certainty_b)
                 << " s_a=[" << zone.s_a_enter << "," << zone.s_a_exit
                 << "] s_b=[" << zone.s_b_enter << "," << zone.s_b_exit
                 << "] center=(" << zone.x << "," << zone.y << ")"
                 << " path_gen_a=" << zone.path_generation_a
                 << " path_gen_b=" << zone.path_generation_b
                 << " source="
                 << forklift_planner::multi_vehicle::
                        futureConflictZoneSourceName(zone.source);
            ROS_INFO("%s", line.str().c_str());
            coordLogWithContext(line.str(), "PREDICT", sim_plan_id_ + 1,
                                -1, -1);
        }
    }

    void logFutureConflictClusters() {
        for (const FutureConflictCluster& cluster :
             future_conflict_clusters_) {
            if (cluster.member_zone_ids.size() < 2) continue;
            std::ostringstream line;
            line << "[FUTURE_CONFLICT_CLUSTER] snapshot="
                 << cluster.horizon_snapshot_id
                 << " cluster=" << cluster.cluster_id
                 << " pair=V" << cluster.vehicle_a << "-V"
                 << cluster.vehicle_b << " members=[";
            for (std::size_t i = 0; i < cluster.member_zone_ids.size(); ++i) {
                if (i > 0) line << ",";
                line << cluster.member_zone_ids[i];
            }
            line << "] merge_reason=";
            for (std::size_t i = 0; i < cluster.merge_reasons.size(); ++i) {
                if (i > 0) line << "|";
                line << forklift_planner::multi_vehicle::
                    futureConflictClusterMergeReasonName(
                        cluster.merge_reasons[i]);
            }
            ROS_INFO("%s", line.str().c_str());
            coordLogWithContext(line.str(), "PREDICT",
                                cluster.horizon_snapshot_id, -1, -1);
        }
    }

    void evaluateFutureClusterArbitrationShadow() {
        cluster_arbitration_shadows_.clear();
        cluster_admission_shadows_.clear();
        cluster_admission_constraints_.clear();
        const auto snapshot = rule_engine_->snapshot();

        auto agentById = [&](int id) -> const VehicleAgent* {
            for (const VehicleAgent& vehicle : agents_) {
                if (vehicle.id == id) return &vehicle;
            }
            return nullptr;
        };

        for (const FutureConflictCluster& cluster :
             future_conflict_clusters_) {
            const VehicleAgent* vehicle_a = agentById(cluster.vehicle_a);
            const VehicleAgent* vehicle_b = agentById(cluster.vehicle_b);
            if (vehicle_a == nullptr || vehicle_b == nullptr ||
                cluster.member_zones.empty()) {
                continue;
            }

            forklift_planner::multi_vehicle::
                ClusterArbitrationShadowContext context;
            const std::pair<int, int> pair{
                std::min(vehicle_a->id, vehicle_b->id),
                std::max(vehicle_a->id, vehicle_b->id)};

            const auto departure = snapshot.departure_clusters.find(pair);
            if (departure != snapshot.departure_clusters.end() &&
                departure->second.active) {
                const auto& commitment = departure->second;
                const VehicleAgent* owner =
                    vehicle_a->id == commitment.owner_id ? vehicle_a :
                    vehicle_b->id == commitment.owner_id ? vehicle_b :
                    nullptr;
                const VehicleAgent* other =
                    vehicle_a->id == commitment.other_id ? vehicle_a :
                    vehicle_b->id == commitment.other_id ? vehicle_b :
                    nullptr;
                if (owner != nullptr && other != nullptr &&
                    owner->path_gen == commitment.owner_path_gen &&
                    other->path_gen == commitment.other_path_gen) {
                    context.departure_cluster_owner_id =
                        forklift_planner::multi_vehicle::
                            futureA1OtherInsideCluster(
                                commitment.intervals, other->path_s)
                            ? other->id
                            : owner->id;
                }
            }

            if (future_a1_commitment_.valid() &&
                (future_a1_commitment_.owner_id == vehicle_a->id ||
                 future_a1_commitment_.owner_id == vehicle_b->id)) {
                const VehicleAgent* owner =
                    future_a1_commitment_.owner_id == vehicle_a->id
                        ? vehicle_a : vehicle_b;
                const VehicleAgent* other =
                    owner == vehicle_a ? vehicle_b : vehicle_a;
                const bool owner_phase_valid =
                    owner->mission_phase == MissionPhase::TO_A1 ||
                    owner->mission_phase == MissionPhase::PICKUP_DWELL;
                bool protects_cluster = false;
                for (const FutureConflictZone& zone :
                     cluster.member_zones) {
                    const bool owner_is_a = zone.vehicle_a == owner->id;
                    const MissionPhase owner_phase =
                        owner_is_a ? zone.phase_a : zone.phase_b;
                    const MissionPhase other_phase =
                        owner_is_a ? zone.phase_b : zone.phase_a;
                    const int owner_generation = owner_is_a
                        ? zone.path_generation_a : zone.path_generation_b;
                    const int other_generation = owner_is_a
                        ? zone.path_generation_b : zone.path_generation_a;
                    if (owner_phase == MissionPhase::TO_B &&
                        other_phase == MissionPhase::TO_A1 &&
                        owner_generation == owner->path_gen + 1 &&
                        other_generation == other->path_gen) {
                        protects_cluster = true;
                        break;
                    }
                }
                if (owner_phase_valid && owner->pending_dropoff_valid &&
                    owner->path_gen ==
                        future_a1_commitment_.owner_path_gen &&
                    other->mission_phase == MissionPhase::TO_A1 &&
                    protects_cluster) {
                    context.future_a1_owner_id = owner->id;
                }
            }

            context.a_a1_departure =
                vehicle_a->a1_departure_committed &&
                vehicle_a->path_s <
                    vehicle_a->a1_departure_priority_until_s - 1e-9;
            context.b_a1_departure =
                vehicle_b->a1_departure_committed &&
                vehicle_b->path_s <
                    vehicle_b->a1_departure_priority_until_s - 1e-9;
            const double terminal_distance = std::max(
                cfg_.target_request_distance, cfg_.target_stop_distance);
            context.a_terminal_docking =
                vehicle_a->active() &&
                vehicle_a->remainingS() <= terminal_distance;
            context.b_terminal_docking =
                vehicle_b->active() &&
                vehicle_b->remainingS() <= terminal_distance;
            context.priority_winner_id =
                rule_engine_->priorityWinner(*vehicle_a, *vehicle_b);

            ClusterReservationShadow result =
                future_cluster_arbitration_shadow_->evaluate(
                    cluster, *vehicle_a, *vehicle_b, context);
            cluster_arbitration_shadows_.push_back(result);

            // Mirrors the existing 0.01 m Future A1/conflict admission
            // buffer.  This value is diagnostic-only and never reaches an
            // action request.
            constexpr double kShadowStopBuffer = 0.01;
            ClusterAdmissionShadow admission =
                future_cluster_admission_shadow_->evaluate(
                    cluster, result, *vehicle_a, *vehicle_b,
                    kShadowStopBuffer);
            admission = future_cluster_admission_shadow_tracker_->update(
                std::move(admission), result, *vehicle_a, *vehicle_b,
                sim_time_);
            cluster_admission_shadows_.push_back(admission);
            ++cluster_admission_evaluations_;
            if (admission.admission_valid) {
                ++cluster_admission_valid_count_;
            }
            if (admission.prevent_zone_mixing) {
                ++cluster_admission_prevent_count_;
            }
            if (admission.zone_mixing_observed) {
                ++cluster_admission_late_mixing_count_;
            }

            const VehicleAgent& constraint_waiter =
                admission.waiter_id == vehicle_a->id
                    ? *vehicle_a : *vehicle_b;
            ClusterAdmissionConstraint constraint =
                cluster_admission_evaluator_->buildConstraint(
                    cluster, admission, result,
                    future_trajectory_cache_, *vehicle_a, *vehicle_b,
                    kShadowStopBuffer, 1.0 / pp_.update_rate,
                    cfg_.max_accel, cfg_.max_decel,
                    constraint_waiter.active()
                        ? curvatureSpeed(constraint_waiter) : 0.0);
            cluster_admission_constraints_.push_back(constraint);
            if (constraint.admission_feasible) {
                ++dynamic_admission_feasible_count_;
            } else if (constraint.admission_reason.rfind(
                           "admission_not_feasible", 0) == 0) {
                ++dynamic_admission_infeasible_count_;
            }
            std::ostringstream dynamic_line;
            dynamic_line << std::fixed << std::setprecision(3)
                << "[DYNAMIC_CLUSTER_ADMISSION_SHADOW] time="
                << readableSimTime(sim_time_)
                << " cluster=" << constraint.cluster_id
                << " holder="
                << (constraint.holder_id >= 0
                        ? "V" + std::to_string(constraint.holder_id)
                        : "none")
                << " waiter="
                << (constraint.waiter_id >= 0
                        ? "V" + std::to_string(constraint.waiter_id)
                        : "none")
                << " cluster_enter_s=" << constraint.cluster_enter_s
                << " cluster_exit_s=" << constraint.cluster_exit_s
                << " earliest_stop_s=" << constraint.earliest_stop_s
                << " required_clearance_s="
                << constraint.required_clearance_s
                << " path_s=" << constraint.evaluated_path_s
                << " speed=" << constraint.evaluated_speed
                << " curvature_speed_limit="
                << constraint.curvature_speed_limit
                << " approach_speed_upper_bound="
                << constraint.approach_speed_upper_bound
                << " required_braking_distance="
                << constraint.required_braking_distance
                << " available_braking_distance="
                << constraint.available_braking_distance
                << " feasible="
                << (constraint.admission_feasible ? "true" : "false")
                << " reason=" << constraint.admission_reason
                << " holder_lifecycle=[";
            for (std::size_t i = 0;
                 i < constraint.holder_lifecycle.size(); ++i) {
                if (i > 0) dynamic_line << ",";
                const auto& lifecycle = constraint.holder_lifecycle[i];
                dynamic_line << missionPhaseName(lifecycle.phase)
                    << ":seg" << lifecycle.segment_id
                    << ":gen" << lifecycle.path_generation
                    << ":s" << lifecycle.cluster_enter_s
                    << "-" << lifecycle.cluster_exit_s;
            }
            dynamic_line << "]";
            ROS_INFO("%s", dynamic_line.str().c_str());
            coordLogWithContext(dynamic_line.str(), "PREDICT",
                                constraint.horizon_snapshot_id, -1, -1);

            int legacy_holder = -1;
            const auto reservation = snapshot.reservations.find(pair);
            if (reservation != snapshot.reservations.end()) {
                legacy_holder = reservation->second.owner_id;
            }
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[CLUSTER_ARBITRATION_SHADOW] time="
                 << readableSimTime(sim_time_)
                 << " snapshot=" << result.horizon_snapshot_id
                 << " cluster_id=" << result.cluster_id
                 << " vehicles=V" << result.vehicle_a << "-V"
                 << result.vehicle_b << " member_zone_ids=[";
            for (std::size_t i = 0; i < result.member_zone_ids.size(); ++i) {
                if (i > 0) line << ",";
                line << result.member_zone_ids[i];
            }
            line << "] holder="
                 << (result.holder_id >= 0
                         ? "V" + std::to_string(result.holder_id) : "none")
                 << " waiter="
                 << (result.waiter_id >= 0
                         ? "V" + std::to_string(result.waiter_id) : "none")
                 << " decision_reason=" << result.decision_reason
                 << " stop_boundary=" << result.stop_boundary_s
                 << " inside_a=" << (result.vehicle_a_inside ? "true" : "false")
                 << " inside_b=" << (result.vehicle_b_inside ? "true" : "false")
                 << " released=" << (result.all_members_cleared ? "true" : "false")
                 << " zone_holders=[";
            for (std::size_t i = 0;
                 i < result.member_zone_holders.size(); ++i) {
                if (i > 0) line << ",";
                line << result.member_zone_holders[i].first << ":";
                const int holder = result.member_zone_holders[i].second;
                line << (holder >= 0 ? "V" + std::to_string(holder)
                                     : "none");
            }
            line << "] mixed_zone_holders="
                 << (result.zone_level_mixed_holders ? "true" : "false")
                 << " legacy_reservation_holder="
                 << (legacy_holder >= 0
                         ? "V" + std::to_string(legacy_holder) : "none");
            ROS_INFO("%s", line.str().c_str());
            coordLogWithContext(line.str(), "PREDICT",
                                result.horizon_snapshot_id, -1, -1);

            std::ostringstream admission_line;
            admission_line << std::fixed << std::setprecision(3)
                << "[CLUSTER_ADMISSION_SHADOW] time="
                << readableSimTime(sim_time_)
                << " cluster_id=" << admission.cluster_id
                << " members=[";
            for (std::size_t i = 0;
                 i < admission.member_zone_ids.size(); ++i) {
                if (i > 0) admission_line << ",";
                admission_line << admission.member_zone_ids[i];
            }
            admission_line << "] holder="
                << (admission.holder_id >= 0
                        ? "V" + std::to_string(admission.holder_id)
                        : "none")
                << " waiter="
                << (admission.waiter_id >= 0
                        ? "V" + std::to_string(admission.waiter_id)
                        : "none")
                << " cluster_enter_s_a="
                << admission.cluster_enter_s_a
                << " cluster_enter_s_b="
                << admission.cluster_enter_s_b
                << " waiter_stop_s=" << admission.waiter_stop_s
                << " prevent_zone_mixing="
                << (admission.prevent_zone_mixing ? "true" : "false")
                << " admission_valid="
                << (admission.admission_valid ? "true" : "false")
                << " waiter_before_entry="
                << (admission.waiter_before_entry ? "true" : "false")
                << " waiter_already_inside="
                << (admission.waiter_already_inside ? "true" : "false")
                << " zone_mixing_observed="
                << (admission.zone_mixing_observed ? "true" : "false")
                << " shadow_lock_active="
                << (admission.shadow_lock_active ? "true" : "false")
                << " holder_change_suppressed="
                << (admission.holder_change_suppressed ? "true" : "false")
                << " lock_created_time="
                << admission.shadow_lock_created_time
                << " decision_reason=" << admission.decision_reason;
            ROS_INFO("%s", admission_line.str().c_str());
            coordLogWithContext(admission_line.str(), "PREDICT",
                                admission.horizon_snapshot_id, -1, -1);
        }
        if (cluster_counterfactual_shadow_enabled_) {
            cluster_admission_counterfactual_simulator_->refresh(
                future_conflict_clusters_, cluster_admission_constraints_,
                cluster_arbitration_shadows_, future_trajectory_cache_,
                agents_, sim_time_);
        }
    }

    void logCounterfactualClusterEvents() {
        for (const auto& event :
             cluster_admission_counterfactual_simulator_->takeEvents()) {
            const auto& status = event.status;
            if (event.event == "RELEASE") {
                ++counterfactual_cluster_releases_;
            } else if (event.event == "RESUME") {
                ++counterfactual_waiter_resumes_;
            } else if (event.event == "WAITER_ENTRY_VIOLATION" ||
                       event.event ==
                           "DYNAMIC_ADMISSION_BOUNDARY_VIOLATION") {
                ++counterfactual_waiter_entry_violations_;
            } else if (event.event ==
                       "LATE_ADMISSION_BRAKING_INFEASIBLE") {
                ++counterfactual_late_braking_events_;
            }
            std::ostringstream line;
            line << std::fixed << std::setprecision(3)
                 << "[CLUSTER_COUNTERFACTUAL] time="
                 << readableSimTime(sim_time_)
                 << " event=" << event.event
                 << " cluster=" << status.cluster_id
                 << " members=[";
            for (std::size_t i = 0;
                 i < status.member_zone_ids.size(); ++i) {
                if (i > 0) line << ",";
                line << status.member_zone_ids[i];
            }
            line << "] holder=V" << status.holder_id
                 << " waiter=V" << status.waiter_id
                 << " waiter_stop_s=" << status.waiter_stop_s
                 << " holder_clear_time=" << status.holder_clear_time
                 << " cluster_release_time="
                 << status.cluster_release_time
                 << " waiter_resume_time=" << status.waiter_resume_time
                 << " waiter_entered_member_zone="
                 << (status.waiter_entered_member_zone ? "true" : "false")
                 << " braking_feasible="
                 << (status.admission_braking_feasible ? "true" : "false");
            ROS_WARN("%s", line.str().c_str());
            coordLogWithContext(line.str(), "COUNTERFACTUAL",
                                status.horizon_snapshot_id, -1, -1);
        }
    }

    void applyClusterAdmissionCounterfactual(double dt) {
        if (!cluster_counterfactual_shadow_enabled_) return;
        counterfactual_shadow_states_ =
            cluster_admission_counterfactual_simulator_->step(
                agents_, sim_time_, dt, cfg_.max_decel);
        logCounterfactualClusterEvents();

        std::map<int, ShadowVehicleState> selected;
        for (const ShadowVehicleState& state :
             counterfactual_shadow_states_) {
            const auto found = selected.find(state.vehicle_id);
            if (found == selected.end() ||
                state.shadow_action == CounterfactualShadowAction::STOP ||
                found->second.shadow_action ==
                    CounterfactualShadowAction::NONE) {
                selected[state.vehicle_id] = state;
            }
        }

        for (const auto& item : selected) {
            VehicleAgent* vehicle = agentById(item.first);
            if (vehicle == nullptr || !vehicle->active()) continue;
            const ShadowVehicleState& state = item.second;
            if (state.shadow_action == CounterfactualShadowAction::STOP) {
                ++dynamic_admission_stop_requests_;
                if (state.action_changed) {
                    ++dynamic_admission_action_changes_;
                }
                const auto decision_key = std::make_tuple(
                    state.cluster_id, state.holder_id, state.waiter_id,
                    vehicle->path_gen,
                    static_cast<long long>(std::llround(
                        state.waiter_stop_s * 1000.0)));
                if (dynamic_admission_decision_log_keys_.insert(
                        decision_key).second) {
                    std::ostringstream decision_line;
                    decision_line << std::fixed << std::setprecision(3)
                        << "[DYNAMIC_CLUSTER_DECISION_SHADOW] time="
                        << readableSimTime(sim_time_)
                        << " cluster=" << state.cluster_id
                        << " holder=V" << state.holder_id
                        << " waiter=V" << state.waiter_id
                        << " baseline_action="
                        << actionName(state.baseline_action)
                        << " baseline_reason=" << vehicle->reason
                        << " constrained_action="
                        << actionName(state.constrained_action)
                        << " action_changed="
                        << (state.action_changed ? "true" : "false")
                        << " baseline_stop_s=not_exposed"
                        << " constraint_stop_s=" << state.waiter_stop_s
                        << " distance_to_stop_s="
                        << state.distance_to_stop_s
                        << " required_braking_distance="
                        << state.required_braking_distance
                        << " early_stop=true";
                    ROS_WARN("%s", decision_line.str().c_str());
                    coordLogWithContext(decision_line.str(),
                                        "COUNTERFACTUAL",
                                        coord_log_plan_id_, -1, -1);
                }
                vehicle->action = VehicleAction::STOP;
                vehicle->requested_action = VehicleAction::STOP;
                vehicle->blocker_id = state.holder_id;
                vehicle->reason = "counterfactual_cluster_wait_V" +
                    std::to_string(state.holder_id);
                vehicle->wait_time = std::max(
                    vehicle->wait_time, state.waiting_duration);
                ++counterfactual_stop_vehicle_ticks_;
                counterfactual_max_cluster_wait_ = std::max(
                    counterfactual_max_cluster_wait_,
                    state.waiting_duration);
                continue;
            }
            if (state.shadow_action == CounterfactualShadowAction::GO &&
                state.cluster_released) {
                force_horizon_refresh_ = true;
            }
            // Counterfactual admission is monotone: it may add a STOP request
            // for the waiter, but it must never relax an action selected by the
            // real RuleEngine.  In particular, the cluster holder remains
            // subject to hard guard, timed conflict, forward-clearance and
            // every existing reservation/priority decision.
        }
    }

    void runTimedConflictShadowComparison() {
        timed_conflict_shadow_reports_.clear();
        if (future_trajectory_cache_.size() != agents_.size() ||
            legacy_prediction_cache_.size() != agents_.size()) {
            return;
        }

        std::vector<forklift_planner::multi_vehicle::ShadowConflictZone> zones;
        auto compareAll = [&]() {
            std::vector<TimedConflictShadowReport> reports;
            for (size_t i = 0; i < agents_.size(); ++i) {
                for (size_t j = i + 1; j < agents_.size(); ++j) {
                    reports.push_back(timed_conflict_shadow_checker_->compare(
                        agents_[i], agents_[j], legacy_prediction_cache_[i],
                        legacy_prediction_cache_[j],
                        future_trajectory_cache_[i],
                        future_trajectory_cache_[j], zones,
                        future_conflict_zones_));
                }
            }
            return reports;
        };

        timed_conflict_shadow_reports_ = compareAll();
        const bool any_collision = std::any_of(
            timed_conflict_shadow_reports_.begin(),
            timed_conflict_shadow_reports_.end(),
            [](const TimedConflictShadowReport& report) {
                return report.old_event.valid || report.new_event.valid;
            });
        if (any_collision) {
            for (const auto& marker :
                 rule_engine_->conflictResourceMarkers(agents_)) {
                if (marker.kind != forklift_planner::multi_vehicle::
                                       ConflictMarkerKind::
                                           POTENTIAL_CONFLICT_ZONE) {
                    continue;
                }
                zones.push_back(
                    forklift_planner::multi_vehicle::ShadowConflictZone{
                        marker.vehicle_a, marker.vehicle_b,
                        marker.zone_index, marker.s_a_enter, marker.s_a_exit,
                        marker.s_b_enter, marker.s_b_exit});
            }
            timed_conflict_shadow_reports_ = compareAll();

            for (const TimedConflictShadowReport& report :
                 timed_conflict_shadow_reports_) {
                if (!report.new_event.valid ||
                    report.new_event.matched_zone < 0 ||
                    report.new_event.future_zone_id < 0) {
                    continue;
                }
                const auto current_it = std::find_if(
                    zones.begin(), zones.end(), [&](const auto& zone) {
                        return zone.vehicle_a == report.vehicle_a &&
                               zone.vehicle_b == report.vehicle_b &&
                               zone.zone_index ==
                                   report.new_event.matched_zone;
                    });
                const auto future_it = std::find_if(
                    future_conflict_zones_.begin(),
                    future_conflict_zones_.end(),
                    [&](const FutureConflictZone& zone) {
                        return zone.future_zone_id ==
                               report.new_event.future_zone_id;
                    });
                if (current_it == zones.end() ||
                    future_it == future_conflict_zones_.end()) {
                    continue;
                }
                const double max_error = std::max(
                    {std::abs(current_it->s_a_enter - future_it->s_a_enter),
                     std::abs(current_it->s_a_exit - future_it->s_a_exit),
                     std::abs(current_it->s_b_enter - future_it->s_b_enter),
                     std::abs(current_it->s_b_exit - future_it->s_b_exit)});
                std::ostringstream line;
                line << std::fixed << std::setprecision(6)
                     << "[FUTURE_ZONE_COMPARE] pair=V" << report.vehicle_a
                     << "-V" << report.vehicle_b
                     << " current_zone="
                     << report.new_event.matched_zone
                     << " future_zone="
                     << report.new_event.future_zone_id
                     << " result="
                     << (max_error <= 1e-9 ? "MATCH" : "DIFFERENCE")
                     << " max_interval_error=" << max_error;
                ROS_INFO("%s", line.str().c_str());
                coordLogWithContext(line.str(), "PREDICT",
                                    sim_plan_id_ + 1, -1, -1);
            }
        }

        for (const TimedConflictShadowReport& report :
             timed_conflict_shadow_reports_) {
            if (!report.old_event.valid && !report.new_event.valid &&
                report.classification ==
                    forklift_planner::multi_vehicle::
                        TimedConflictShadowClass::MATCH) {
                continue;
            }
            std::ostringstream line;
            line << std::fixed << std::setprecision(2)
                 << "[TIMED_CONFLICT_SHADOW] pair=V" << report.vehicle_a
                 << "-V" << report.vehicle_b
                 << " class="
                 << forklift_planner::multi_vehicle::
                        timedConflictShadowClassName(report.classification)
                 << " old_collision=" << report.old_event.valid
                 << " new_collision=" << report.new_event.valid
                 << " old_first=";
            if (report.old_event.valid) {
                line << report.old_event.first_t
                     << " old_last=" << report.old_event.last_t
                     << " old_zone=" << report.old_event.matched_zone;
            } else {
                line << "none old_last=none old_zone=none";
            }
            line << " new_first=";
            if (report.new_event.valid) {
                line << report.new_event.first_t
                     << " new_last=" << report.new_event.last_t
                     << " duration=" << report.new_event.overlap_duration
                     << " samples=" << report.new_event.overlap_samples
                     << " new_zone=";
                if (report.new_event.matched_zone >= 0) {
                    line << report.new_event.matched_zone;
                } else {
                    line << "none";
                }
                line << " future_zone=";
                if (report.new_event.future_zone_id >= 0) {
                    line << report.new_event.future_zone_id;
                } else {
                    line << "none";
                }
                line << " segment_a=" << report.new_event.segment_a
                     << " phase_a="
                     << missionPhaseName(report.new_event.phase_a)
                     << " certainty_a="
                     << forklift_planner::multi_vehicle::futureCertaintyName(
                            report.new_event.certainty_a)
                     << " segment_b=" << report.new_event.segment_b
                     << " phase_b="
                     << missionPhaseName(report.new_event.phase_b)
                     << " certainty_b="
                     << forklift_planner::multi_vehicle::futureCertaintyName(
                            report.new_event.certainty_b);
            } else {
                line << "none new_last=none duration=0 samples=0 "
                        "new_zone=none";
            }
            ROS_INFO("%s", line.str().c_str());
            coordLogWithContext(line.str(), "PREDICT", sim_plan_id_ + 1,
                                -1, -1);
        }
    }

    void publishFutureConflictZoneShadowMarkers() {
        visualization_msgs::MarkerArray markers;
        const ros::Time stamp = ros::Time::now();
        for (const auto& key : future_conflict_zone_marker_keys_) {
            visualization_msgs::Marker remove;
            remove.header.frame_id = pp_.frame_id;
            remove.header.stamp = stamp;
            remove.ns = key.first;
            remove.id = key.second;
            remove.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(remove);
        }
        future_conflict_zone_marker_keys_.clear();

        if (cfg_.show_prediction_conflicts) {
            int marker_id = 0;
            for (const FutureConflictZone& zone : future_conflict_zones_) {
                visualization_msgs::Marker point;
                point.header.frame_id = pp_.frame_id;
                point.header.stamp = stamp;
                point.ns = "future_conflict_zone_shadow";
                point.id = marker_id++;
                point.type = visualization_msgs::Marker::SPHERE;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.position.x = zone.x;
                point.pose.position.y = zone.y;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.10;
                point.scale.y = 0.10;
                point.scale.z = 0.025;
                point.color = zone.source ==
                                      forklift_planner::multi_vehicle::
                                          FutureConflictZoneSource::
                                              CURRENT_TRACK
                    ? rgba(0.10f, 0.75f, 0.95f, 0.65f)
                    : rgba(0.15f, 0.95f, 0.55f, 0.75f);
                point.lifetime = ros::Duration(
                    std::max(0.2, 1.5 * rb_horizon_refresh_period_));
                markers.markers.push_back(point);
                future_conflict_zone_marker_keys_.push_back(
                    {point.ns, point.id});

                visualization_msgs::Marker label = point;
                label.ns = "future_conflict_zone_shadow_label";
                label.id = marker_id++;
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.pose.position.z = 0.12;
                label.scale.x = 0.0;
                label.scale.y = 0.0;
                label.scale.z = 0.075;
                label.color = rgba(0.80f, 1.0f, 0.90f, 1.0f);
                std::ostringstream text;
                text << std::fixed << std::setprecision(2)
                     << "FZ" << zone.future_zone_id << " V"
                     << zone.vehicle_a << "-V" << zone.vehicle_b << " "
                     << missionPhaseName(zone.phase_a) << "#"
                     << zone.segment_id_a << " x "
                     << missionPhaseName(zone.phase_b) << "#"
                     << zone.segment_id_b << "\nA[" << zone.s_a_enter
                     << "," << zone.s_a_exit << "] B["
                     << zone.s_b_enter << "," << zone.s_b_exit << "]";
                label.text = text.str();
                markers.markers.push_back(label);
                future_conflict_zone_marker_keys_.push_back(
                    {label.ns, label.id});
            }
        }
        if (!markers.markers.empty()) horizon_marker_pub_.publish(markers);
    }

    void publishFutureConflictClusterShadowMarkers() {
        visualization_msgs::MarkerArray markers;
        const ros::Time stamp = ros::Time::now();
        for (const auto& key : future_conflict_cluster_marker_keys_) {
            visualization_msgs::Marker remove;
            remove.header.frame_id = pp_.frame_id;
            remove.header.stamp = stamp;
            remove.ns = key.first;
            remove.id = key.second;
            remove.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(remove);
        }
        future_conflict_cluster_marker_keys_.clear();

        if (cfg_.show_prediction_conflicts) {
            for (const FutureConflictCluster& cluster :
                 future_conflict_clusters_) {
                if (cluster.member_zones.empty()) continue;
                double center_x = 0.0;
                double center_y = 0.0;
                for (const FutureConflictZone& zone : cluster.member_zones) {
                    center_x += zone.x;
                    center_y += zone.y;
                }
                center_x /= static_cast<double>(cluster.member_zones.size());
                center_y /= static_cast<double>(cluster.member_zones.size());

                visualization_msgs::Marker point;
                point.header.frame_id = pp_.frame_id;
                point.header.stamp = stamp;
                point.ns = "future_conflict_cluster_shadow";
                point.id = cluster.cluster_id;
                point.type = visualization_msgs::Marker::SPHERE;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.position.x = center_x;
                point.pose.position.y = center_y;
                point.pose.position.z = 0.035;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.14;
                point.scale.y = 0.14;
                point.scale.z = 0.04;
                point.color = rgba(0.78f, 0.20f, 1.0f, 0.82f);
                point.lifetime = ros::Duration(
                    std::max(0.2, 1.5 * rb_horizon_refresh_period_));
                markers.markers.push_back(point);
                future_conflict_cluster_marker_keys_.push_back(
                    {point.ns, point.id});

                visualization_msgs::Marker label = point;
                label.ns = "future_conflict_cluster_shadow_label";
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.pose.position.z = 0.20;
                label.scale.x = 0.0;
                label.scale.y = 0.0;
                label.scale.z = 0.080;
                label.color = rgba(0.95f, 0.80f, 1.0f, 1.0f);
                std::ostringstream text;
                text << "FC" << cluster.cluster_id << " V"
                     << cluster.vehicle_a << "-V" << cluster.vehicle_b
                     << " zones=[";
                for (std::size_t i = 0;
                     i < cluster.member_zone_ids.size(); ++i) {
                    if (i > 0) text << ",";
                    text << cluster.member_zone_ids[i];
                }
                text << "]\nreason=";
                if (cluster.merge_reasons.empty()) {
                    text << "SINGLE_ZONE";
                } else {
                    for (std::size_t i = 0;
                         i < cluster.merge_reasons.size(); ++i) {
                        if (i > 0) text << "|";
                        text << forklift_planner::multi_vehicle::
                            futureConflictClusterMergeReasonName(
                                cluster.merge_reasons[i]);
                    }
                }
                label.text = text.str();
                markers.markers.push_back(label);
                future_conflict_cluster_marker_keys_.push_back(
                    {label.ns, label.id});
            }
        }
        if (!markers.markers.empty()) horizon_marker_pub_.publish(markers);
    }

    void publishFutureClusterArbitrationShadowMarkers() {
        visualization_msgs::MarkerArray markers;
        const ros::Time stamp = ros::Time::now();
        for (const auto& key :
             future_cluster_arbitration_marker_keys_) {
            visualization_msgs::Marker remove;
            remove.header.frame_id = pp_.frame_id;
            remove.header.stamp = stamp;
            remove.ns = key.first;
            remove.id = key.second;
            remove.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(remove);
        }
        future_cluster_arbitration_marker_keys_.clear();

        if (cfg_.show_prediction_conflicts) {
            int marker_id = 0;
            for (const ClusterReservationShadow& shadow :
                 cluster_arbitration_shadows_) {
                const FutureConflictCluster* source = nullptr;
                for (const FutureConflictCluster& cluster :
                     future_conflict_clusters_) {
                    if (cluster.cluster_id == shadow.cluster_id &&
                        cluster.horizon_snapshot_id ==
                            shadow.horizon_snapshot_id &&
                        cluster.vehicle_a == shadow.vehicle_a &&
                        cluster.vehicle_b == shadow.vehicle_b) {
                        source = &cluster;
                        break;
                    }
                }
                if (source == nullptr || source->member_zones.empty()) {
                    continue;
                }
                double center_x = 0.0;
                double center_y = 0.0;
                for (const FutureConflictZone& zone : source->member_zones) {
                    center_x += zone.x;
                    center_y += zone.y;
                }
                center_x /= static_cast<double>(source->member_zones.size());
                center_y /= static_cast<double>(source->member_zones.size());

                visualization_msgs::Marker point;
                point.header.frame_id = pp_.frame_id;
                point.header.stamp = stamp;
                point.ns = "future_cluster_arbitration_shadow";
                point.id = marker_id++;
                point.type = visualization_msgs::Marker::SPHERE;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.position.x = center_x;
                point.pose.position.y = center_y;
                point.pose.position.z = 0.055;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.10;
                point.scale.y = 0.10;
                point.scale.z = 0.06;
                point.color = rgba(1.0f, 0.62f, 0.05f, 0.92f);
                point.lifetime = ros::Duration(
                    std::max(0.2, 1.5 * rb_horizon_refresh_period_));
                markers.markers.push_back(point);
                future_cluster_arbitration_marker_keys_.push_back(
                    {point.ns, point.id});

                visualization_msgs::Marker label = point;
                label.ns = "future_cluster_arbitration_shadow_label";
                label.id = marker_id++;
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.pose.position.z = 0.30;
                label.scale.x = 0.0;
                label.scale.y = 0.0;
                label.scale.z = 0.075;
                label.color = rgba(1.0f, 0.86f, 0.35f, 1.0f);
                std::ostringstream text;
                text << std::fixed << std::setprecision(2)
                     << "CA" << shadow.cluster_id
                     << " H=V" << shadow.holder_id
                     << " W=V" << shadow.waiter_id
                     << " n=" << shadow.member_zone_ids.size()
                     << "\nstop=" << shadow.stop_boundary_s
                     << " " << shadow.decision_reason;
                label.text = text.str();
                markers.markers.push_back(label);
                future_cluster_arbitration_marker_keys_.push_back(
                    {label.ns, label.id});
            }
        }
        if (!markers.markers.empty()) horizon_marker_pub_.publish(markers);
    }

    void publishFutureClusterAdmissionShadowMarkers() {
        visualization_msgs::MarkerArray markers;
        const ros::Time stamp = ros::Time::now();
        for (const auto& key : future_cluster_admission_marker_keys_) {
            visualization_msgs::Marker remove;
            remove.header.frame_id = pp_.frame_id;
            remove.header.stamp = stamp;
            remove.ns = key.first;
            remove.id = key.second;
            remove.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(remove);
        }
        future_cluster_admission_marker_keys_.clear();

        if (cfg_.show_prediction_conflicts) {
            int marker_id = 0;
            for (const ClusterAdmissionShadow& admission :
                 cluster_admission_shadows_) {
                const FutureConflictCluster* source = nullptr;
                for (const FutureConflictCluster& cluster :
                     future_conflict_clusters_) {
                    if (cluster.cluster_id == admission.cluster_id &&
                        cluster.horizon_snapshot_id ==
                            admission.horizon_snapshot_id &&
                        cluster.vehicle_a == admission.vehicle_a &&
                        cluster.vehicle_b == admission.vehicle_b) {
                        source = &cluster;
                        break;
                    }
                }
                if (source == nullptr || source->member_zones.empty()) {
                    continue;
                }
                double center_x = 0.0;
                double center_y = 0.0;
                for (const FutureConflictZone& zone : source->member_zones) {
                    center_x += zone.x;
                    center_y += zone.y;
                }
                center_x /= static_cast<double>(source->member_zones.size());
                center_y /= static_cast<double>(source->member_zones.size());

                visualization_msgs::Marker point;
                point.header.frame_id = pp_.frame_id;
                point.header.stamp = stamp;
                point.ns = "future_cluster_admission_shadow";
                point.id = marker_id++;
                point.type = visualization_msgs::Marker::SPHERE;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.position.x = center_x;
                point.pose.position.y = center_y;
                point.pose.position.z = 0.075;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.08;
                point.scale.y = 0.08;
                point.scale.z = 0.08;
                point.color = admission.admission_valid
                    ? rgba(0.10f, 0.95f, 0.55f, 0.95f)
                    : admission.prevent_zone_mixing
                        ? rgba(1.0f, 0.25f, 0.75f, 0.95f)
                        : rgba(0.55f, 0.55f, 0.55f, 0.75f);
                point.lifetime = ros::Duration(
                    std::max(0.2, 1.5 * rb_horizon_refresh_period_));
                markers.markers.push_back(point);
                future_cluster_admission_marker_keys_.push_back(
                    {point.ns, point.id});

                visualization_msgs::Marker label = point;
                label.ns = "future_cluster_admission_shadow_label";
                label.id = marker_id++;
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.pose.position.z = 0.39;
                label.scale.x = 0.0;
                label.scale.y = 0.0;
                label.scale.z = 0.070;
                label.color = rgba(0.75f, 1.0f, 0.85f, 1.0f);
                std::ostringstream text;
                text << std::fixed << std::setprecision(2)
                     << "ADMIT C" << admission.cluster_id
                     << " H=V" << admission.holder_id
                     << " W=V" << admission.waiter_id
                     << "\nentry=(" << admission.cluster_enter_s_a
                     << "," << admission.cluster_enter_s_b << ")"
                     << " stop=" << admission.waiter_stop_s
                     << " "
                     << (admission.admission_valid ? "VALID" : "SHADOW");
                label.text = text.str();
                markers.markers.push_back(label);
                future_cluster_admission_marker_keys_.push_back(
                    {label.ns, label.id});
            }
        }
        if (!markers.markers.empty()) horizon_marker_pub_.publish(markers);
    }

    void publishFutureClusterCounterfactualMarkers() {
        visualization_msgs::MarkerArray markers;
        const ros::Time stamp = ros::Time::now();
        for (const auto& key : future_cluster_counterfactual_marker_keys_) {
            visualization_msgs::Marker remove;
            remove.header.frame_id = pp_.frame_id;
            remove.header.stamp = stamp;
            remove.ns = key.first;
            remove.id = key.second;
            remove.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(remove);
        }
        future_cluster_counterfactual_marker_keys_.clear();

        if (cluster_counterfactual_shadow_enabled_ &&
            cfg_.show_prediction_conflicts) {
            int marker_id = 0;
            for (const auto& status :
                 cluster_admission_counterfactual_simulator_->statuses()) {
                const FutureConflictCluster* source = nullptr;
                for (const FutureConflictCluster& cluster :
                     future_conflict_clusters_) {
                    if (cluster.vehicle_a == status.vehicle_a &&
                        cluster.vehicle_b == status.vehicle_b &&
                        cluster.member_zone_ids == status.member_zone_ids) {
                        source = &cluster;
                        break;
                    }
                }
                if (source == nullptr || source->member_zones.empty()) continue;
                double x = 0.0;
                double y = 0.0;
                for (const FutureConflictZone& zone : source->member_zones) {
                    x += zone.x;
                    y += zone.y;
                }
                x /= static_cast<double>(source->member_zones.size());
                y /= static_cast<double>(source->member_zones.size());

                visualization_msgs::Marker point;
                point.header.frame_id = pp_.frame_id;
                point.header.stamp = stamp;
                point.ns = "future_cluster_counterfactual";
                point.id = marker_id++;
                point.type = visualization_msgs::Marker::SPHERE;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.position.x = x;
                point.pose.position.y = y;
                point.pose.position.z = 0.11;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.10;
                point.scale.y = 0.10;
                point.scale.z = 0.10;
                point.color = status.active
                    ? rgba(0.90f, 0.15f, 1.0f, 0.95f)
                    : rgba(0.40f, 0.40f, 0.40f, 0.70f);
                point.lifetime = ros::Duration(
                    std::max(0.2, 1.5 * rb_horizon_refresh_period_));
                markers.markers.push_back(point);
                future_cluster_counterfactual_marker_keys_.push_back(
                    {point.ns, point.id});

                visualization_msgs::Marker label = point;
                label.ns = "future_cluster_counterfactual_label";
                label.id = marker_id++;
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.pose.position.z = 0.48;
                label.scale.x = 0.0;
                label.scale.y = 0.0;
                label.scale.z = 0.070;
                label.color = rgba(1.0f, 0.75f, 1.0f, 1.0f);
                std::ostringstream text;
                text << std::fixed << std::setprecision(2)
                     << "CF C" << status.cluster_id
                     << " H=V" << status.holder_id
                     << " W=V" << status.waiter_id
                     << " stop=" << status.waiter_stop_s
                     << "\nrelease=";
                if (status.cluster_release_time >= 0.0) {
                    text << status.cluster_release_time;
                } else {
                    text << "pending";
                }
                label.text = text.str();
                markers.markers.push_back(label);
                future_cluster_counterfactual_marker_keys_.push_back(
                    {label.ns, label.id});
            }
        }
        if (!markers.markers.empty()) horizon_marker_pub_.publish(markers);
    }

    void publishFutureShadowConflictMarkers() {
        visualization_msgs::MarkerArray markers;
        const ros::Time stamp = ros::Time::now();
        for (const auto& key : future_shadow_conflict_marker_keys_) {
            visualization_msgs::Marker remove;
            remove.header.frame_id = pp_.frame_id;
            remove.header.stamp = stamp;
            remove.ns = key.first;
            remove.id = key.second;
            remove.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(remove);
        }
        future_shadow_conflict_marker_keys_.clear();

        if (cfg_.show_prediction_conflicts) {
            int marker_id = 0;
            for (const TimedConflictShadowReport& report :
                 timed_conflict_shadow_reports_) {
                if (!report.new_event.valid) continue;
                visualization_msgs::Marker point;
                point.header.frame_id = pp_.frame_id;
                point.header.stamp = stamp;
                point.ns = "future_shadow_conflict";
                point.id = marker_id++;
                point.type = visualization_msgs::Marker::SPHERE;
                point.action = visualization_msgs::Marker::ADD;
                point.pose.position.x = report.new_event.x;
                point.pose.position.y = report.new_event.y;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.12;
                point.scale.y = 0.12;
                point.scale.z = 0.04;
                point.color = rgba(0.75f, 0.15f, 0.95f, 0.80f);
                point.lifetime = ros::Duration(
                    std::max(0.2, 1.5 * rb_horizon_refresh_period_));
                markers.markers.push_back(point);
                future_shadow_conflict_marker_keys_.push_back(
                    {point.ns, point.id});

                visualization_msgs::Marker label = point;
                label.ns = "future_shadow_conflict_label";
                label.id = marker_id++;
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.pose.position.z = 0.18;
                label.scale.x = 0.0;
                label.scale.y = 0.0;
                label.scale.z = 0.10;
                label.color = rgba(0.95f, 0.85f, 1.0f, 1.0f);
                std::ostringstream text;
                text << std::fixed << std::setprecision(2)
                     << "SHADOW V" << report.vehicle_a << "-V"
                     << report.vehicle_b << " t="
                     << report.new_event.first_t << "~"
                     << report.new_event.last_t << "\n"
                     << missionPhaseName(report.new_event.phase_a) << "#"
                     << report.new_event.segment_a << " / "
                     << missionPhaseName(report.new_event.phase_b) << "#"
                     << report.new_event.segment_b;
                label.text = text.str();
                markers.markers.push_back(label);
                future_shadow_conflict_marker_keys_.push_back(
                    {label.ns, label.id});
            }
        }
        if (!markers.markers.empty()) horizon_marker_pub_.publish(markers);
    }

    void publishFutureMissionMarkers() {
        visualization_msgs::MarkerArray markers;
        const ros::Time stamp = ros::Time::now();

        for (const auto& key : future_mission_marker_keys_) {
            visualization_msgs::Marker remove;
            remove.header.frame_id = pp_.frame_id;
            remove.header.stamp = stamp;
            remove.ns = key.first;
            remove.id = key.second;
            remove.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(remove);
        }
        future_mission_marker_keys_.clear();

        int marker_id = 0;
        for (const FutureMissionTrajectory& trajectory :
             future_trajectory_cache_) {
            for (const auto& segment : trajectory.plan.segments) {
                const char* certainty_name =
                    forklift_planner::multi_vehicle::futureCertaintyName(
                        segment.certainty);
                std::string marker_ns = "future_mission_";
                if (segment.certainty == FutureCertainty::COMMITTED) {
                    marker_ns += "committed";
                } else if (segment.certainty == FutureCertainty::PREVIEW) {
                    marker_ns += "preview";
                } else {
                    marker_ns += "unknown";
                }

                std_msgs::ColorRGBA color;
                if (segment.certainty == FutureCertainty::COMMITTED) {
                    color = rgba(0.10f, 0.85f, 0.20f, 0.85f);
                } else if (segment.certainty == FutureCertainty::PREVIEW) {
                    color = rgba(0.10f, 0.65f, 1.00f, 0.55f);
                } else {
                    color = rgba(0.65f, 0.65f, 0.65f, 0.80f);
                }

                std::vector<const forklift_planner::multi_vehicle::FutureSample*>
                    segment_samples;
                for (const auto& sample : trajectory.samples) {
                    if (sample.segment_id == segment.segment_id) {
                        segment_samples.push_back(&sample);
                    }
                }
                if (segment_samples.empty()) continue;

                visualization_msgs::Marker geometry;
                geometry.header.frame_id = pp_.frame_id;
                geometry.header.stamp = stamp;
                geometry.ns = marker_ns;
                geometry.id = marker_id++;
                geometry.action = visualization_msgs::Marker::ADD;
                geometry.pose.orientation.w = 1.0;
                geometry.color = color;
                if (segment.type == FutureSegmentType::MOTION &&
                    segment_samples.size() >= 2) {
                    geometry.type = visualization_msgs::Marker::LINE_STRIP;
                    geometry.scale.x = segment.certainty ==
                            FutureCertainty::PREVIEW ? 0.012 : 0.018;
                    for (const auto* sample : segment_samples) {
                        geometry_msgs::Point point;
                        point.x = sample->pose.x;
                        point.y = sample->pose.y;
                        point.z = 0.13;
                        geometry.points.push_back(point);
                    }
                } else {
                    geometry.type = visualization_msgs::Marker::SPHERE;
                    geometry.pose.position.x = segment_samples.front()->pose.x;
                    geometry.pose.position.y = segment_samples.front()->pose.y;
                    geometry.pose.position.z = 0.13;
                    geometry.scale.x = 0.055;
                    geometry.scale.y = 0.055;
                    geometry.scale.z = 0.025;
                }
                markers.markers.push_back(geometry);
                future_mission_marker_keys_.push_back(
                    {geometry.ns, geometry.id});

                visualization_msgs::Marker label;
                label.header = geometry.header;
                label.ns = marker_ns + "_label";
                label.id = marker_id++;
                label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                label.action = visualization_msgs::Marker::ADD;
                label.pose.position.x = segment_samples.front()->pose.x;
                label.pose.position.y = segment_samples.front()->pose.y;
                label.pose.position.z = 0.20;
                label.pose.orientation.w = 1.0;
                label.scale.z = 0.045;
                label.color = color;
                label.color.a = 1.0;
                label.text = "V" + std::to_string(trajectory.plan.vehicle_id) +
                    " " + missionPhaseName(segment.phase) + " " +
                    certainty_name;
                markers.markers.push_back(label);
                future_mission_marker_keys_.push_back({label.ns, label.id});
            }
        }
        if (!markers.markers.empty()) horizon_marker_pub_.publish(markers);
    }

    bool simulationPlanNeedsRefresh() const {
        if (!sim_plan_valid_ || force_horizon_refresh_) return true;
        if (future_a1_commitment_.valid()) {
            const VehicleAgent* owner =
                agentById_c(future_a1_commitment_.owner_id);
            if (owner == nullptr ||
                owner->path_gen != future_a1_commitment_.owner_path_gen) {
                return true;
            }
        }
        if (sim_plan_cursor_ >= sim_plan_frames_.size()) return true;
        if (sim_plan_cursor_ >= static_cast<size_t>(rb_horizon_refresh_)) {
            return true;
        }
        const SimPlanFrame& frame = sim_plan_frames_[sim_plan_cursor_];
        if (frame.agents.size() != agents_.size()) return true;
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (frame.agents[i].path_gen != agents_[i].path_gen ||
                frame.agents[i].mode != agents_[i].mode) {
                return true;
            }
        }
        return false;
    }

    bool executeSimulationPlanSample() {
        if (simulationPlanNeedsRefresh()) return false;
        setCoordLogContext("REAL", sim_plan_id_,
                           static_cast<int>(sim_plan_cursor_), -1);
        const SimPlanFrame& frame = sim_plan_frames_[sim_plan_cursor_];
        rule_engine_->restore(frame.rule_state);
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            const SimPlannedAgentDecision& d = frame.agents[i];
            v.action = d.action;
            v.requested_action = d.requested_action;
            v.blocker_id = d.blocker_id;
            v.wait_time = d.wait_time;
            v.action_hold_remaining = d.action_hold_remaining;
            v.cycle_break_immunity = d.cycle_break_immunity;
            v.deadlock_breaker = d.deadlock_breaker;
            v.deadlock_breaker_hold = d.deadlock_breaker_hold;
            v.reason = d.reason;
        }
        ++sim_plan_cursor_;
        return true;
    }

    // Freeze the already-required A1->B leg before cloning the world for a
    // rolling-horizon rollout. This is real-state task preparation, not a
    // simulated assignment: prepareDropoffLeg() only fills the pending_* fields
    // and leaves the currently executing B->A1 leg ACTIVE/TO_A1. Once the
    // rollout reaches A1, its existing state-machine path consumes that frozen
    // information after PICKUP_DWELL via activatePreparedDropoffLeg().
    void prepareA1DropoffPreviewsForHorizon() {
        if (!cfg_.use_a1_cycle || sim_mode_) return;

        for (VehicleAgent& v : agents_) {
            if (!targetEnabled(v.id) ||
                v.mode != VehicleMode::ACTIVE ||
                v.mission_phase != MissionPhase::TO_A1 ||
                v.leg_target != LegTargetKind::A1 ||
                v.track.empty() ||
                v.pending_dropoff_valid) {
                continue;
            }

            allocator_->prepareDropoffLeg(v, agents_);
        }
    }

    // 滚动时域发布:推演 rb_horizon_ 秒,把每车时间参数化轨迹发到 /traj_i(刷新=滚动)。
    // hold 车(整段不动)发单点轨迹(size=1)作静止标志 → 控制器 idle 不控制。
    // 滚动时域发布：重新推演并覆盖每辆车的短时轨迹。
    void publishHorizon() {
        std::vector<sandbox_msgs::Trajectory> trajs;
        std::vector<bool> hold;

        // Give the cloned world only future tasks that have already been
        // selected and stored in real state. The rollout itself remains unable
        // to create a new B target while sim_mode_ is true.
        prepareA1DropoffPreviewsForHorizon();
        
        // =======世界模型推演，传入（预测时长，预测得到每辆车未来轨迹，预测得到每辆车未来是否保持静止）=============
        if (cfg_.real_mode) {
            rollWorldModel(rb_horizon_, trajs, hold);
        } else {
            buildSimulationHorizonPlan(trajs, hold);
        }


        const ros::Time now = ros::Time::now();
        const double tnow = now.toSec();
        std::vector<bool> guard(trajs.size(), false);
        if (cfg_.real_mode) {
            guard = realHardGuard();
        }

          //==================（对每辆车先在Rviz中删除旧轨迹）==========================
        visualization_msgs::MarkerArray arr;
        for (size_t i = 0; i < trajs.size(); ++i) {
            visualization_msgs::Marker del;
            del.header.frame_id = pp_.frame_id;
            del.header.stamp = now;
            del.ns = "horizon_traj";
            del.id = static_cast<int>(i);
            del.action = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(del);

            const VehicleAgent& v = agents_[i];

            // ========（若车辆正在休眠，则发布单点静止轨迹，不刷新轨迹）=============
            if ((v.mode == VehicleMode::DWELL || v.dwell_remaining > 1e-6) &&
                !v.pending_dropoff_valid) {
                if (i < traj_pubs_.size()) {
                    sandbox_msgs::Trajectory hold;
                    hold.target = static_cast<int>(i);
                    hold.header.frame_id = "world";
                    hold.header.stamp = now;

                    const RoughWp p = poseForCollision(v, v.path_s);
                    sandbox_msgs::TrajectoryPoint tp;
                    tp.x = p.x;
                    tp.y = p.y;
                    tp.yaw = p.theta;
                    tp.velocity = 0.0;
                    tp.time = 0.0;
                    hold.points.push_back(tp);

                    traj_pubs_[i].publish(hold);
            }

        continue;
}
            // ============（在实车条件下，若车辆位置摆放不正确，则不发布指令）========================
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;


            // ============（若车辆急停、超时、保护，则轨迹退化为单点）========================
            const bool stale = cfg_.real_mode && cfg_.real_pose_timeout > 0.0 &&
                               (tnow - rb_last_seen_[i]) > cfg_.real_pose_timeout;
            if (rb_estop_ || stale || guard[i]) hold[i] = true;

            trajs[i].header.stamp = now;
            if (hold[i] && !trajs[i].points.empty()) {
                trajs[i].points.resize(1);
            }

            //=================（画新的Rviz轨迹，并发布轨迹给控制器）=======================
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = now;
            m.ns = "horizon_traj";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.02;
            m.color = agents_[i].color;
            m.color.a = 0.9;
            for (const auto& p : trajs[i].points) {
                geometry_msgs::Point gp;
                gp.x = p.x;
                gp.y = p.y;
                gp.z = 0.09;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);

            if (i < traj_pubs_.size()) {
                traj_pubs_[i].publish(trajs[i]);
            }
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    bool publishFullTrajectories() {
        std::vector<sandbox_msgs::Trajectory> trajs;
        std::vector<bool> hold;
        rollWorldModel(rb_full_horizon_, trajs, hold);   // 按全程上限推演一次(尾部静止点稍后裁掉)
        const ros::Time now = ros::Time::now();
        visualization_msgs::MarkerArray arr;
        bool all_done = true;
        for (size_t i = 0; i < trajs.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            if (agents_[i].track.empty()) continue;      // 无路径的车不计入(不卡总进度)
            if (one_shot_done_[i]) continue;             // 已发过的车不重发(latched 已在控制器手上)
            if (cfg_.real_mode && !real_pose_ok_[i]) {                     // 动捕未就位 → 这辆暂不发,标记未完成下拍补
                ROS_WARN_THROTTLE(1.0, "[real][one_shot] V%zu 动捕未就位 → 暂不发轨迹(就位后补发)", i);
                all_done = false;
                continue;
            }
            trimTrailingStationary(trajs[i]);            // 裁掉到点后的尾部静止点,只留一个停止点
            trajs[i].header.stamp = now;
            if (i < traj_pubs_.size()) traj_pubs_[i].publish(trajs[i]);             // latch 发一次,控制器自主跟到底
            one_shot_done_[i] = true;
            logFullTraj(i, trajs[i]);
            // RViz:整条轨迹画成该车颜色 LINE_STRIP(ns=horizon_traj,沿用现有显示)
            if (trajs[i].points.size() >= 2) {
                visualization_msgs::Marker m;
                m.header.frame_id = pp_.frame_id; m.header.stamp = now;
                m.ns = "horizon_traj"; m.id = (int)i;
                m.type = visualization_msgs::Marker::LINE_STRIP;
                m.action = visualization_msgs::Marker::ADD;
                m.pose.orientation.w = 1.0; m.scale.x = 0.02;
                m.color = agents_[i].color; m.color.a = 0.9;
                for (const auto& p : trajs[i].points) {
                    geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = 0.09;
                    m.points.push_back(gp);
                }
                arr.markers.push_back(m);
            }
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
        return all_done;
    }

    // 急停:给每车发单点 hold 轨迹(当前真实位姿,v=0)。外部 pure_pursuit 收到 size=1 的轨迹后
    // 锁在该点附近停车(不再前进)。一次性纯盲跟下这是唯一的软件急停手段(协调层已不干预纵向)。
    void publishHoldAll() {
        const ros::Time now = ros::Time::now();
        int sent = 0;
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;          // 没真实位姿就别发垃圾点
            sandbox_msgs::Trajectory t;
            t.target = (int)i; t.header.frame_id = "world"; t.header.stamp = now;
            sandbox_msgs::TrajectoryPoint p;
            p.x = real_x_[i]; p.y = real_y_[i]; p.yaw = real_yaw_[i];
            p.velocity = 0.0; p.time = 0.0;
            t.points.push_back(p);
            traj_pubs_[i].publish(t);                 // latch:控制器停在原地
            ++sent;
        }
        ROS_ERROR("[real][one_shot] *** 急停:已对 %d 辆车发单点 hold 轨迹 → 控制器停车 ***", sent);
    }

    // 急停解除:从当前真实位置重新推演全程并重发。不能简单重发原轨迹——外部控制器按轨迹 time
    // 跟踪,重发会把 start_time_ 归零、车在中途却从 t=0 等起 → 卡死。重置 one_shot 标志,让主循环
    // 调 publishFullTrajectories 重新 rollWorldModel(从 realAdvance 同步好的当前 path_s 出发,
    // time=0 即对齐"现在")。
    void resumeFromEstop() {
        std::fill(one_shot_done_.begin(), one_shot_done_.end(), false);
        one_shot_published_ = false;
        ROS_WARN("[real][one_shot] *** 急停解除:从当前真实位置重新推演全程,准备重发 ***");
    }

    // 裁掉到达终点后的尾部静止点(velocity≈0):全程推演为留余量按大 horizon 跑,到点后会拖一长串
    // 原地不动的点,白白撑大轨迹消息。保留末尾一个 velocity=0 的停止点,控制器据此判 reached。
    void trimTrailingStationary(sandbox_msgs::Trajectory& t) {
        if (t.points.size() < 3) return;
        int k = (int)t.points.size() - 1;
        while (k > 0 && std::fabs(t.points[k].velocity) < 1e-3) --k;  // 最后一个仍在动的点
        const int stop = std::min(k + 1, (int)t.points.size() - 1);   // 紧随其后的停止点(在终点)
        sandbox_msgs::TrajectoryPoint sp = t.points[stop];
        sp.velocity = 0.0;
        t.points.resize(k + 1);
        t.points.push_back(sp);
    }

    // 一次性轨迹诊断日志:点数 / 时长 / 路径全长 / 有无倒车段 / 是否真跑到终点(否则 full_horizon 太短)。
    void logFullTraj(size_t i, const sandbox_msgs::Trajectory& t) {
        if (t.points.empty()) return;
        const double dur = t.points.back().time;
        bool has_rev = false;
        for (const auto& p : t.points) if (p.velocity < -1e-3) { has_rev = true; break; }
        const double len = agents_[i].track.empty() ? 0.0 : agents_[i].track.length();
        const bool reached = std::fabs(t.points.back().velocity) < 1e-3;
        ROS_WARN("[real][one_shot] V%zu 发整条轨迹 → /traj_%zu: 点数=%zu 时长=%.1fs 全长=%.2fm 倒车段=%s%s",
                 i, i, t.points.size(), dur, len, has_rev ? "有" : "无",
                 reached ? "" : "  ⚠ 末点仍在动:full_horizon 太短,加大 ~full_horizon");
    }

    //==============《任务指派与路径生成函数》===================================
    void updateDwellAndTasks(double dt) {
        for (VehicleAgent& v : agents_) {

            //*************** 1. 若该车未启用，则车状态一直置为STOP *******
            if (!targetEnabled(v.id)) {
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.current_speed = 0.0;
                continue;
            }

            // A1-cycle is a two-leg logistics state machine. A1 is not a map
            // slot, so current_slot remains the last physical B slot until the
            // loaded A1->B leg reaches its destination.
            if (cfg_.use_a1_cycle) {
                if (v.mode == VehicleMode::NEED_TASK) {
                    if (sim_mode_) continue;
                    const int old_gen = v.path_gen;
                    allocator_->assignNextTask(v, agents_);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }

                if (v.mode != VehicleMode::DWELL) continue;

                v.dwell_remaining = std::max(0.0, v.dwell_remaining - dt);
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.current_speed = 0.0;
                if (v.dwell_remaining > 1e-9) continue;

                // A rollout may predict arrival at A1, but it must not invent
                // a new B assignment. It may, however, activate the departure
                // plan that was prepared and reserved at the real A1 arrival.
                if (sim_mode_) {
                    if (v.mission_phase == MissionPhase::PICKUP_DWELL &&
                        v.pending_dropoff_valid) {
                        allocator_->activatePreparedDropoffLeg(
                            v, /*emit_log=*/false);
                    } else {
                        v.dwell_remaining = 0.0;
                    }
                    continue;
                }

                if (v.mission_phase == MissionPhase::PICKUP_DWELL) {
                    const int old_gen = v.path_gen;
                    allocator_->assignDropoffLeg(v, agents_);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }

                if (v.mission_phase == MissionPhase::WAIT_DROPOFF_TASK) {
                    const int old_gen = v.path_gen;
                    allocator_->assignDropoffLeg(v, agents_);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }

                if (v.mission_phase == MissionPhase::UNLOAD_DWELL) {
                    const bool completed_transport = v.loaded;
                    v.loaded = false;
                    if (completed_transport) ++v.task_count;
                    if (one_shot_) {
                        v.action = VehicleAction::STOP;
                        v.requested_action = VehicleAction::STOP;
                        v.reason = "one_shot_complete";
                        continue;
                    }

                    v.mission_phase = MissionPhase::TO_A1;
                    v.leg_target = LegTargetKind::A1;
                    v.mode = VehicleMode::NEED_TASK;
                    const int old_gen = v.path_gen;
                    allocator_->assignPickupLeg(v);
                    if (v.path_gen != old_gen) force_horizon_refresh_ = true;
                    continue;
                }
                continue;
            }

            // ********2. 若该车状态为需要任务，则尝试指派任务 ************
            // NEED_TASK 的车每拍重试派活——分配可能因"当下所有路都与在途车对穿"而暂时失败,
            // 但别车一移动局面就变,必须重试,否则车永久饿死(实测 6 车卡死的根因之一)。
            if (v.mode == VehicleMode::NEED_TASK) {
                if (sim_mode_) continue;
                allocator_->assignNextTask(v, agents_);         //***关键任务指派函数和路径生成 ******/
                continue;
            }


            //********** 3.若车不为休眠状态，则直接跳过 *********************
            if (v.mode != VehicleMode::DWELL) continue;

            //**********4.反之，若车在休眠状态，则停止动作，计算倒计睡眠时间，同时根据设置执行操作*/
            v.dwell_remaining = std::max(0.0, v.dwell_remaining - dt);
            v.action = VehicleAction::STOP;
            v.requested_action = VehicleAction::STOP;
            v.current_speed = 0.0;
                    
                //********* 4.1 若车已过睡眠时间，并且为一次性规划，则跳过不再派发任务 ******/
            if (v.dwell_remaining <= 1e-9) {
                if (sim_mode_) {
                    v.dwell_remaining = 0.0;
                    continue;
                }
                if (one_shot_) {        // 一次性 demo:到达目标后永久停,不再派活(8车各跑一程 A→B)
                    v.action = VehicleAction::STOP;
                    v.requested_action = VehicleAction::STOP;
                    continue;
                }

                //******** 4.2 若车已过睡眠时间，并且为不间断跑，则切换速度，并派发任务 ******/
                const VehicleAgent before_task = v;
                v.loaded = !v.loaded;
                allocator_->assignNextTask(v, agents_);
                
                // 前瞻预测性避免("提前预料到就避免"):若这次发车会在 H 内导致持续死锁,就**错峰**
                // ——撤销发车、在车位再等一会(不堵路),让别车先过那段窄区,稍后再试。仅在真预测到
                // 死锁时才扣(精准,非广撒网);连扣上限防极端饥饿。整块仅真实模式执行(sim 内不递归)。
                if (!sim_mode_ && v.mode == VehicleMode::ACTIVE) {
                    if (predict_holds_[v.id] < 6 && simPredictsDeadlock()) {
                        const int hold_id = v.id;
                        v = before_task;
                        v.mode = VehicleMode::DWELL;
                        v.dwell_remaining = 2.0;
                        v.action = VehicleAction::STOP;
                        v.requested_action = VehicleAction::STOP;
                        v.current_speed = 0.0;
                        v.reason = "predict_hold";
                        ++predict_holds_[hold_id];
                    } else {
                        predict_holds_[v.id] = 0;      // 真发车了 → 清零连扣计数
                    }
                }
            }
        }
    }

    //==============================================

    void handleLegArrival(VehicleAgent& v) {
        v.current_speed = 0.0;
        v.wait_time = 0.0;
        v.mode = VehicleMode::DWELL;
        v.action = VehicleAction::STOP;
        v.requested_action = VehicleAction::STOP;

        if (cfg_.use_a1_cycle && v.leg_target == LegTargetKind::A1) {
            const MissionPhase old_phase = v.mission_phase;
            // A1 is a virtual pickup point, not a B slot. Do not write its
            // virtual id into current_slot or visited_slots_.
            v.loaded = false;
            v.mission_phase = MissionPhase::PICKUP_DWELL;
            v.dwell_remaining = cfg_.pickup_dwell_time;
            v.reason = "pickup_dwell";
            if (!sim_mode_) {
                char line[300];
                std::snprintf(
                    line, sizeof(line),
                    "[A1_ARRIVAL] V%d phase=%s->%s pending_dropoff=%d "
                    "dropoff_slot=%d track_size=%zu path_gen=%d",
                    v.id, missionPhaseName(old_phase),
                    missionPhaseName(v.mission_phase),
                    v.pending_dropoff_valid ? 1 : 0,
                    v.pending_dropoff_slot,
                    v.pending_dropoff_track.path().size(), v.path_gen);
                coordLog(line);
            }
            // Simulation-only behavior for now: choose and reserve B at the
            // real A1 arrival so the next rollout can contain 5 s pickup plus
            // the future A1 exit. Do not change the proven real-vehicle task
            // timing until the simulation design has been validated.
            if (!sim_mode_ && !cfg_.real_mode) {
                if (allocator_->prepareDropoffLeg(v, agents_)) {
                    force_horizon_refresh_ = true;
                } else {
                    ROS_ERROR("[multi_patrol][A1 EXIT PREPARE FAILED] V%d "
                              "has no reservable A1->B task at pickup start",
                              v.id);
                }
            }
            return;
        }

        v.current_slot = v.target_slot;
        if (v.current_slot >= 0 &&
            v.current_slot < static_cast<int>(visited_slots_.size())) {
            visited_slots_[static_cast<size_t>(v.current_slot)] = true;
        }

        if (cfg_.use_a1_cycle) {
            v.mission_phase = MissionPhase::UNLOAD_DWELL;
            v.dwell_remaining = cfg_.unload_dwell_time;
            v.reason = "unload_dwell";
        } else {
            ++v.task_count;
            v.dwell_remaining = cfg_.dwell_time;
            v.reason = "dwell";
        }
    }

    void advanceVehicles(double dt) {
        std::vector<double> next_s(agents_.size(), 0.0);
        std::vector<double> next_speed(agents_.size(), 0.0);
        std::vector<double> planned_s(agents_.size(), 0.0);
        std::vector<bool> blocked(agents_.size(), false);

        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            next_s[i] = v.path_s;
            planned_s[i] = v.path_s;
            next_speed[i] = v.current_speed;
            if (!v.active()) continue;

            // 规划速度=动作档,再被曲率限速卡住(与实车 coord_speed 同一套,sim 才能真实验证)。
            const double desired_speed = std::min(rule_engine_->speedForAction(v.action),
                                                  curvatureSpeed(v));
            next_speed[i] = limitedSpeed(v.current_speed, desired_speed, dt);
            next_s[i] = std::min(v.track.length(), v.path_s + next_speed[i] * dt);
            planned_s[i] = next_s[i];
        }

        auto plannedS = [&](size_t idx) {
            return planned_s[idx];
        };

        auto bodyAt = [&](size_t idx, double path_s) {
            const RoughWp pose = poseForCollision(agents_[idx], path_s);
            return forklift_planner::multi_vehicle::makeBody(pose, mp_, 0.0);
        };

        auto overlapsAt = [&](size_t i, double s_i, size_t j, double s_j) {
            return forklift_planner::multi_vehicle::overlaps(bodyAt(i, s_i),
                                                             bodyAt(j, s_j));
        };

        auto canPlace = [&](size_t idx, double candidate_s) {
            for (size_t k = 0; k < agents_.size(); ++k) {
                if (k == idx) continue;
                if (agents_[k].mode != VehicleMode::ACTIVE &&
                    agents_[k].mode != VehicleMode::DWELL) {
                    continue;
                }
                if (overlapsAt(idx, candidate_s, k, plannedS(k))) return false;
            }
            return true;
        };

        auto canPlaceIgnoringPair = [&](size_t idx, double candidate_s,
                                        size_t pair_other) {
            for (size_t k = 0; k < agents_.size(); ++k) {
                if (k == idx || k == pair_other) continue;
                if (agents_[k].mode != VehicleMode::ACTIVE &&
                    agents_[k].mode != VehicleMode::DWELL) {
                    continue;
                }
                if (overlapsAt(idx, candidate_s, k, plannedS(k))) return false;
            }
            return true;
        };

        auto tryClearBlocker = [&](size_t idx, int blocker_id) {
            VehicleAgent& v = agents_[idx];
            if (!v.active()) return false;
            if (next_s[idx] > v.path_s + 1e-9) return false;

            const double creep_speed =
                limitedSpeed(v.current_speed,
                             rule_engine_->speedForAction(VehicleAction::CREEP),
                             dt);
            const double candidate_s =
                std::min(v.track.length(), v.path_s + creep_speed * dt);
            if (candidate_s <= v.path_s + 1e-9) return false;
            if (!canPlace(idx, candidate_s)) return false;

            blocked[idx] = false;
            next_speed[idx] = creep_speed;
            next_s[idx] = candidate_s;
            planned_s[idx] = candidate_s;
            v.action = VehicleAction::CREEP;
            v.requested_action = VehicleAction::CREEP;
            v.reason = "clear_blocker_V" + std::to_string(blocker_id);
            return true;
        };

        auto blockVehicle = [&](size_t idx) {
            if (!agents_[idx].active()) return false;
            const bool changed =
                !blocked[idx] || std::abs(planned_s[idx] - agents_[idx].path_s) > 1e-9 ||
                next_speed[idx] > 1e-9;
            blocked[idx] = true;
            planned_s[idx] = agents_[idx].path_s;
            next_s[idx] = agents_[idx].path_s;
            next_speed[idx] = 0.0;
            return changed;
        };

        auto resolvePlannedOverlaps = [&]() {
        const size_t max_guard_iterations =
            std::max<size_t>(4, agents_.size() * agents_.size() * 2);
        for (size_t iter = 0; iter < max_guard_iterations; ++iter) {
            bool changed = false;
            bool any_overlap = false;

            for (size_t i = 0; i < agents_.size(); ++i) {
                if (agents_[i].mode != VehicleMode::ACTIVE &&
                    agents_[i].mode != VehicleMode::DWELL) {
                    continue;
                }

                for (size_t j = i + 1; j < agents_.size(); ++j) {
                    if (agents_[j].mode != VehicleMode::ACTIVE &&
                        agents_[j].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (!overlapsAt(i, plannedS(i), j, plannedS(j))) {
                        continue;
                    }

                    any_overlap = true;
                    const bool i_active = agents_[i].active();
                    const bool j_active = agents_[j].active();
                    if (i_active && !j_active) {
                        changed = blockVehicle(i) || changed;
                    } else if (!i_active && j_active) {
                        changed = blockVehicle(j) || changed;
                    } else if (i_active && j_active) {
                        const bool i_moves =
                            next_s[i] > agents_[i].path_s + 1e-9;
                        const bool j_moves =
                            next_s[j] > agents_[j].path_s + 1e-9;
                        const bool i_only_safe =
                            i_moves &&
                            !overlapsAt(i, next_s[i], j, agents_[j].path_s);
                        const bool j_only_safe =
                            j_moves &&
                            !overlapsAt(i, agents_[i].path_s, j, next_s[j]);
                        const bool both_next_safe =
                            i_moves && j_moves &&
                            !overlapsAt(i, next_s[i], j, next_s[j]) &&
                            canPlaceIgnoringPair(i, next_s[i], j) &&
                            canPlaceIgnoringPair(j, next_s[j], i);
                        const bool both_stop_safe =
                            !overlapsAt(i, agents_[i].path_s, j, agents_[j].path_s);

                        if (both_next_safe) {
                            const bool changed_i =
                                blocked[i] ||
                                std::abs(planned_s[i] - next_s[i]) > 1e-9;
                            const bool changed_j =
                                blocked[j] ||
                                std::abs(planned_s[j] - next_s[j]) > 1e-9;
                            blocked[i] = false;
                            blocked[j] = false;
                            planned_s[i] = next_s[i];
                            planned_s[j] = next_s[j];
                            changed = changed_i || changed_j || changed;
                        } else if (i_only_safe && !j_only_safe) {
                            changed = blockVehicle(j) || changed;
                            tryClearBlocker(i, agents_[j].id);
                        } else if (!i_only_safe && j_only_safe) {
                            changed = blockVehicle(i) || changed;
                            tryClearBlocker(j, agents_[i].id);
                        } else if (i_only_safe && j_only_safe) {
                            const int winner = rule_engine_->priorityWinner(
                                agents_[i], agents_[j]);
                            if (winner == agents_[i].id) {
                                changed = blockVehicle(j) || changed;
                                tryClearBlocker(i, agents_[j].id);
                            } else if (winner == agents_[j].id) {
                                changed = blockVehicle(i) || changed;
                                tryClearBlocker(j, agents_[i].id);
                            } else {
                                // Tie-break rule disabled: no winner, stop both.
                                changed = blockVehicle(i) || changed;
                                changed = blockVehicle(j) || changed;
                            }
                        } else if (both_stop_safe) {
                            changed = blockVehicle(i) || changed;
                            changed = blockVehicle(j) || changed;
                        } else {
                            const int winner = rule_engine_->priorityWinner(
                                agents_[i], agents_[j]);
                            if (winner == agents_[i].id) {
                                changed = blockVehicle(j) || changed;
                                if (!tryClearBlocker(i, agents_[j].id)) {
                                    changed = blockVehicle(i) || changed;
                                }
                            } else if (winner == agents_[j].id) {
                                changed = blockVehicle(i) || changed;
                                if (!tryClearBlocker(j, agents_[i].id)) {
                                    changed = blockVehicle(j) || changed;
                                }
                            } else {
                                // Tie-break rule disabled: no winner, stop both.
                                changed = blockVehicle(i) || changed;
                                changed = blockVehicle(j) || changed;
                            }
                        }
                    }
                    if (!sim_mode_) {  // 前瞻仿真中只要其物理挡停效果,不计数/不打日志
                        ++hard_guard_events_;
                        hard_guard_pairs_.insert(
                            {std::min(agents_[i].id, agents_[j].id),
                             std::max(agents_[i].id, agents_[j].id)});
                        if (first_guard_tick_ == 0) first_guard_tick_ = tick_count_;
                        ROS_ERROR_THROTTLE(
                            1.0,
                            "[multi_patrol] hard collision guard: V%d vs V%d; "
                            "minimal stop applied",
                            agents_[i].id, agents_[j].id);
                    }
                }
            }

            if (!any_overlap || !changed) {
                break;
            }
        }
        };

        resolvePlannedOverlaps();

        bool any_active_motion = false;
        bool any_blocked_active = false;
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (!agents_[i].active()) continue;
            any_blocked_active = any_blocked_active || blocked[i];
            if (!blocked[i] && planned_s[i] > agents_[i].path_s + 1e-9) {
                any_active_motion = true;
            }
        }

        if (cfg_.enable_stall_release &&
            (!any_active_motion || any_blocked_active)) {
            // 真实开车原则(规格§15)：只往「确实空着」的地方挪——前方若有别人的车身
            // (含安全余量)就老实等，绝不往里蹭。故每次轻推都先用 conflict_margin 校验
            // 候选车身是否清空；不清空就不推。这样既能放走"前方其实空、只是过度谨慎"
            // 的车(恢复流动)，又不会把车顶进别人(避免 stall_release 蹭出 V1↔V3 那种
            // 残留重叠 → hard_collision_guard 反复触发)。真·僵死留给 deadlock_reverse。
            const double cm = cfg_.conflict_margin * 0.5;
            auto clearAheadWithMargin = [&](size_t idx, double cand_s) {
                const auto me = forklift_planner::multi_vehicle::makeBody(
                    poseForCollision(agents_[idx], cand_s), mp_, cm);
                for (size_t k = 0; k < agents_.size(); ++k) {
                    if (k == idx) continue;
                    if (agents_[k].mode != VehicleMode::ACTIVE &&
                        agents_[k].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    const auto other = forklift_planner::multi_vehicle::makeBody(
                        poseForCollision(agents_[k], plannedS(k)), mp_, cm);
                    if (forklift_planner::multi_vehicle::overlaps(me, other)) {
                        return false;
                    }
                }
                return true;
            };
            for (size_t i = 0; i < agents_.size(); ++i) {
                VehicleAgent& v = agents_[i];
                if (!v.active()) continue;
                if (!blocked[i] && planned_s[i] > v.path_s + 1e-9) continue;
                const double creep_speed =
                    limitedSpeed(v.current_speed,
                                 rule_engine_->speedForAction(VehicleAction::CREEP),
                                 dt);
                const double candidate_s =
                    std::min(v.track.length(), v.path_s + creep_speed * dt);
                if (candidate_s <= v.path_s + 1e-9) continue;
                // 只往清空的空间挪；前方被占(含余量)则不推，老实等。
                if (!clearAheadWithMargin(i, candidate_s)) continue;
                blocked[i] = false;
                next_speed[i] = creep_speed;
                next_s[i] = candidate_s;
                planned_s[i] = candidate_s;
                v.action = VehicleAction::CREEP;
                v.requested_action = VehicleAction::CREEP;
                v.reason = "global_stall_release";
            }
            resolvePlannedOverlaps();
        }

        if (cfg_.enable_deadlock_reverse) {
            auto motionHeading = [](const VehicleAgent& v) {
                constexpr double kPi = 3.14159265358979323846;
                double h = v.track.poseAtS(v.path_s).theta;
                if (v.track.typeAtS(v.path_s) == WpType::REVERSE) h += kPi;
                return h;
            };
            auto forwardBlocker = [&](size_t idx) -> int {
                const VehicleAgent& v = agents_[idx];
                const double fwd_s = std::min(
                    v.track.length(),
                    v.path_s + rule_engine_->speedForAction(VehicleAction::CREEP) * dt);
                for (size_t k = 0; k < agents_.size(); ++k) {
                    if (k == idx) continue;
                    if (agents_[k].mode != VehicleMode::ACTIVE &&
                        agents_[k].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (overlapsAt(idx, fwd_s, k, plannedS(k))) {
                        return static_cast<int>(k);
                    }
                }
                return -1;
            };

            constexpr double kReverseWait = 3.0;  // s stuck before backing out
            constexpr double kHeadOnDot = -0.5;   // headings nearly opposite
            bool any_reversed = false;
            for (size_t i = 0; i < agents_.size(); ++i) {
                VehicleAgent& v = agents_[i];
                if (!v.active() || v.wait_time < kReverseWait) continue;
                const int bk = forwardBlocker(i);
                if (bk < 0) continue;
                const VehicleAgent& b = agents_[bk];
                if (b.active() && planned_s[bk] > b.path_s + 1e-9) continue;
                const double hv = motionHeading(v);
                const double hb = motionHeading(b);
                const double dot = std::cos(hv) * std::cos(hb) +
                                   std::sin(hv) * std::sin(hb);
                if (dot > kHeadOnDot) continue;
                if (rule_engine_->priorityWinner(v, b) == v.id) continue;
                const double rev_speed =
                    rule_engine_->speedForAction(VehicleAction::CREEP);
                const double candidate_s =
                    std::max(0.0, v.path_s - rev_speed * dt);
                if (candidate_s >= v.path_s - 1e-9) continue;  // at path start
                if (!canPlace(i, candidate_s)) continue;       // someone behind
                blocked[i] = false;
                next_speed[i] = 0.0;
                next_s[i] = candidate_s;
                planned_s[i] = candidate_s;
                v.action = VehicleAction::CREEP;
                v.requested_action = VehicleAction::CREEP;
                v.reason = "deadlock_reverse_V" + std::to_string(bk);
                any_reversed = true;
            }
            if (any_reversed) resolvePlannedOverlaps();
        }

        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            if (!v.active()) continue;

            if (blocked[i]) {
                v.current_speed = 0.0;
                v.action = VehicleAction::STOP;
                v.requested_action = VehicleAction::STOP;
                v.reason = "hard_collision_guard";
                // §9 完整性:把"被谁的车身物理挡住"也回填为等待边,供 RuleEngine 的
                // 等待图(resolveDeadlock)检测环。否则硬护栏导致的互堵看不见、破不了。
                const double fwd_s = std::min(
                    v.track.length(),
                    v.path_s + rule_engine_->speedForAction(VehicleAction::CREEP) * dt);
                v.blocker_id = -1;
                for (size_t k = 0; k < agents_.size(); ++k) {
                    if (k == i) continue;
                    if (agents_[k].mode != VehicleMode::ACTIVE &&
                        agents_[k].mode != VehicleMode::DWELL) {
                        continue;
                    }
                    if (overlapsAt(i, fwd_s, k, plannedS(k))) {
                        v.blocker_id = agents_[k].id;
                        break;
                    }
                }
                continue;
            }

            v.current_speed = next_speed[i];
            v.path_s = next_s[i];

            if (v.a1_departure_committed &&
                v.path_s >= v.a1_departure_priority_until_s - 1e-9) {
                v.a1_departure_committed = false;
                if (!sim_mode_) {
                    ROS_INFO("[multi_patrol][A1 EXIT PRIORITY RELEASE] V%d "
                             "s=%.3f threshold=%.3f",
                             v.id, v.path_s,
                             v.a1_departure_priority_until_s);
                }
            }

            if (v.path_s >= v.track.length() - 1e-9) {
                handleLegArrival(v);
                // 批处理(长测)里关掉每次到位的 INFO——24h×8车×数千任务=2万+条,会把
                // 关键的"首撞/首楔"现场 dump 在 rosout 滚动里冲掉。实时/RViz 模式保留。
                if (cfg_batch_ticks_ == 0 && !sim_mode_) {
                    const std::string destination =
                        v.leg_target == LegTargetKind::A1
                            ? "A1"
                            : "B" + std::to_string(v.current_slot);
                    ROS_INFO("[multi_patrol] tick=%llu sim_t=%.2f V%d arrived %s; "
                             "dwell %.2fs; load=%s phase=%d",
                             static_cast<unsigned long long>(tick_count_), sim_time_,
                             v.id, destination.c_str(), v.dwell_remaining,
                             v.loaded ? "loaded" : "empty",
                             static_cast<int>(v.mission_phase));
                }
            }
        }
    }

    void logAgentStatus() {
        const ros::Time now = ros::Time::now();
        for (size_t i = 0; i < agents_.size(); ++i) {
            const VehicleAgent& v = agents_[i];
            const bool changed =
                v.mode != last_logged_mode_[i] ||
                v.action != last_logged_action_[i] ||
                v.reason != last_logged_reason_[i] ||
                v.blocker_id != last_logged_blocker_[i] ||
                v.task_count != last_logged_task_count_[i] ||
                v.mission_phase != last_logged_mission_phase_[i];
            const bool stopped_active =
                v.mode == VehicleMode::ACTIVE && v.action == VehicleAction::STOP;
            const bool periodic =
                stopped_active &&
                (last_status_log_time_[i].isZero() ||
                 (now - last_status_log_time_[i]).toSec() >= 2.0);
            if (!changed && !periodic) continue;

            const double length = v.track.empty() ? 0.0 : v.track.length();
            const double rem = v.track.empty() ? 0.0 : v.remainingS();
            char buf[512];
            const std::string readable_time = readableSimTime(sim_time_);
            std::snprintf(
                buf, sizeof(buf),
                "[multi_patrol][state] tick=%llu sim_t=%s V%d "
                "mode=%s phase=%s action=%s reason=%s "
                "blocker=%d task=%d slot=%d->%d s=%.3f/%.3f rem=%.3f "
                "speed=%.3f wait=%.2f dwell=%.2f",
                static_cast<unsigned long long>(tick_count_),
                readable_time.c_str(),
                v.id, modeName(v.mode), missionPhaseName(v.mission_phase),
                actionName(v.action),
                v.reason.empty() ? "-" : v.reason.c_str(), v.blocker_id,
                v.task_count, v.current_slot, v.target_slot, v.path_s,
                length, rem, v.current_speed, v.wait_time,
                v.dwell_remaining);
            const std::string console_line = contextualLog(
                buf, "REAL", coord_log_plan_id_, coord_log_frame_id_, -1);
            ROS_INFO("%s", console_line.c_str());
            coordLog(buf);

            last_logged_mode_[i] = v.mode;
            last_logged_action_[i] = v.action;
            last_logged_reason_[i] = v.reason;
            last_logged_blocker_[i] = v.blocker_id;
            last_logged_task_count_[i] = v.task_count;
            last_logged_mission_phase_[i] = v.mission_phase;
            last_status_log_time_[i] = now;
        }
    }

    // TEMPORARY: dump relative geometry of any vehicle stuck (speed~0, wait>5s)
    // against its blocker, to classify head-on vs follower-misclassification vs
    // priority circularity. Remove once the V6/V7 deadlock root cause is fixed.
    void logStuckDiagnostics() {
        const ros::Time now = ros::Time::now();
        auto motionHeading = [](const VehicleAgent& v) {
            constexpr double kPi = 3.14159265358979323846;
            double h = v.track.poseAtS(v.path_s).theta;
            if (v.track.typeAtS(v.path_s) == WpType::REVERSE) h += kPi;
            return h;
        };
        for (size_t i = 0; i < agents_.size(); ++i) {
            const VehicleAgent& v = agents_[i];
            if (v.mode != VehicleMode::ACTIVE) continue;
            if (v.current_speed > 1e-3 || v.wait_time < 5.0) continue;
            if (!last_diag_time_[i].isZero() &&
                (now - last_diag_time_[i]).toSec() < 3.0) continue;
            last_diag_time_[i] = now;

            const RoughWp pv = v.track.poseAtS(v.path_s);
            const int wt = static_cast<int>(v.track.typeAtS(v.path_s));
            if (v.blocker_id < 0 ||
                v.blocker_id >= static_cast<int>(agents_.size())) {
                ROS_DEBUG("[DIAG stuck] V%d wait=%.1f reason=%s wp=%d "
                         "pose=(%.3f,%.3f) blocker=none",
                         v.id, v.wait_time, v.reason.c_str(), wt, pv.x, pv.y);
                continue;
            }
            const VehicleAgent& b = agents_[v.blocker_id];
            const RoughWp pb = b.track.poseAtS(b.path_s);
            const double hv = motionHeading(v);
            const double hb = motionHeading(b);
            const double dx = pb.x - pv.x;
            const double dy = pb.y - pv.y;
            const double dot = std::cos(hv) * std::cos(hb) +
                               std::sin(hv) * std::sin(hb);
            const double fwd = dx * std::cos(hv) + dy * std::sin(hv);
            const double lat = std::abs(-dx * std::sin(hv) + dy * std::cos(hv));
            const double gap = std::hypot(dx, dy) - mp_.vehicle_length;
            ROS_DEBUG("[DIAG stuck] V%d wait=%.1f reason=%s wp=%d | "
                     "blkV%d(act=%s spd=%.3f wp=%d) dot=%.2f fwd=%.3f lat=%.3f "
                     "gap=%.3f vw=%.3f",
                     v.id, v.wait_time, v.reason.c_str(), wt, b.id,
                     actionName(b.action), b.current_speed,
                     static_cast<int>(b.track.typeAtS(b.path_s)), dot, fwd, lat,
                     gap, mp_.vehicle_width);
        }
    }

    //==================== 最重要的节拍函数======================================
    // Diagnostic-only branch for the unresolved A1 case: another vehicle is
    // already physically inside the visible portion of a prepared A1 exit.
    // This method never issues a recovery action.
    void diagnoseA1ExitIntrusions() {
        if (sim_mode_ || cfg_.real_mode || !cfg_.use_a1_cycle) return;

        std::set<std::pair<int, int>> current;
        const double step = std::max(0.02, cfg_.prediction_step);
        const double footprint_margin = 0.5 * cfg_.conflict_margin;

        for (const VehicleAgent& owner : agents_) {
            const bool pending_owner =
                owner.mode == VehicleMode::DWELL &&
                owner.mission_phase == MissionPhase::PICKUP_DWELL &&
                owner.pending_dropoff_valid &&
                !owner.pending_dropoff_track.empty();
            const bool active_owner =
                owner.active() && owner.a1_departure_committed;
            if (!pending_owner && !active_owner) {
                continue;
            }

            const double dwell_before_motion =
                pending_owner ? owner.dwell_remaining : 0.0;
            const double motion_time =
                std::max(0.0, rb_horizon_ - dwell_before_motion);
            if (motion_time <= 1e-9) continue;

            VehicleAgent preview = owner;
            if (pending_owner) {
                preview.track = owner.pending_dropoff_track;
                preview.path_s = 0.0;
                preview.current_speed = 0.0;
            }
            preview.mode = VehicleMode::ACTIVE;

            struct ExitSample {
                double t;
                double s;
                forklift_planner::multi_vehicle::OBB body;
            };
            std::vector<ExitSample> exit_samples;
            exit_samples.push_back(ExitSample{
                dwell_before_motion, preview.path_s,
                forklift_planner::multi_vehicle::makeBody(
                    preview.track.poseAtS(preview.path_s), mp_,
                    footprint_margin)});
            double elapsed = 0.0;
            while (elapsed < motion_time - 1e-9 &&
                   preview.path_s < preview.track.length() - 1e-9) {
                const double sample_dt = std::min(step, motion_time - elapsed);
                const double desired = std::min(
                    rule_engine_->speedForAction(VehicleAction::NOMINAL),
                    curvatureSpeed(preview));
                preview.current_speed =
                    limitedSpeed(preview.current_speed, desired, sample_dt);
                preview.path_s = std::min(
                    preview.track.length(),
                    preview.path_s + preview.current_speed * sample_dt);
                elapsed += sample_dt;
                exit_samples.push_back(ExitSample{
                    dwell_before_motion + elapsed, preview.path_s,
                    forklift_planner::multi_vehicle::makeBody(
                        preview.track.poseAtS(preview.path_s), mp_,
                        footprint_margin)});
            }

            for (const VehicleAgent& other : agents_) {
                if (other.id == owner.id ||
                    (other.mode != VehicleMode::ACTIVE &&
                     other.mode != VehicleMode::DWELL) ||
                    other.track.empty()) {
                    continue;
                }
                const forklift_planner::multi_vehicle::OBB other_body =
                    forklift_planner::multi_vehicle::makeBody(
                        poseForCollision(other, other.path_s), mp_,
                        footprint_margin);
                const ExitSample* hit = nullptr;
                for (const ExitSample& sample : exit_samples) {
                    if (forklift_planner::multi_vehicle::overlaps(
                            sample.body, other_body)) {
                        hit = &sample;
                        break;
                    }
                }
                if (hit == nullptr) continue;

                const std::pair<int, int> key{owner.id, other.id};
                current.insert(key);
                if (active_a1_exit_intrusions_.count(key) == 0) {
                    char line[420];
                    std::snprintf(
                        line, sizeof(line),
                        "[multi_patrol][A1 EXIT INTRUSION ENTER] owner=V%d "
                        "target=B%d blocker=V%d dwell_remaining=%.2fs "
                        "visible_exit_t=%.2fs owner_exit_s=%.3f "
                        "blocker_phase=%d blocker_s=%.3f action=DIAG_ONLY",
                        owner.id,
                        pending_owner ? owner.pending_dropoff_slot
                                      : owner.target_slot,
                        other.id, dwell_before_motion, hit->t, hit->s,
                        static_cast<int>(other.mission_phase), other.path_s);
                    const std::string console_line = contextualLog(
                        line, "REAL", coord_log_plan_id_,
                        coord_log_frame_id_, -1);
                    ROS_WARN("%s", console_line.c_str());
                    coordLog(line);
                }
            }
        }

        for (const auto& old : active_a1_exit_intrusions_) {
            if (current.count(old) != 0) continue;
            char line[180];
            std::snprintf(
                line, sizeof(line),
                "[multi_patrol][A1 EXIT INTRUSION CLEAR] owner=V%d blocker=V%d",
                old.first, old.second);
            const std::string console_line = contextualLog(
                line, "REAL", coord_log_plan_id_, coord_log_frame_id_, -1);
            ROS_WARN("%s", console_line.c_str());
            coordLog(line);
        }
        active_a1_exit_intrusions_.swap(current);
    }

    void tick(const ros::TimerEvent&) {
        const double dt = 1.0 / pp_.update_rate;        //控制周期与仿真系统推移周期一致
        ++tick_count_;          //记录系统运行了多少隔周期
        sim_time_ += dt;        //仿真时间增加一个固定时间步长
        setCoordLogContext("REAL", sim_plan_id_, coord_log_frame_id_, -1);


        // 1. 实车模式---未摆放好姿态模式
        if (cfg_.real_mode) {
            if (!rb_started_) {
                marker_pub_->publish(
                    agents_, visited_slots_, rule_engine_->conflicts(),
                    marker_pub_->hasSubscribers()
                        ? rule_engine_->conflictResourceMarkers(agents_)
                        : std::vector<forklift_planner::multi_vehicle::
                              ConflictMarker>{});   //发布车辆、地图
                publishRealTrailMarkers();  //发布真实车身尾迹
                logPlacementStatus();       //打印摆车状态
                return;
            }

        //2. 实车模式---选择一次性发布完整轨迹
            if (rb_one_shot_traj_) {
                if (rb_estop_ && !rb_estop_prev_) publishHoldAll();         //刚刚进入急停，给所有车发单点轨迹
                else if (!rb_estop_ && rb_estop_prev_) resumeFromEstop();   //刚刚解除急停，就从当前位置重新发轨迹
                rb_estop_prev_ = rb_estop_;

                if (!rb_estop_) {               //若没有急停
                    if (!one_shot_published_) one_shot_published_ = publishFullTrajectories();
                    updateDwellAndTasks(dt);        //检测到没有发送轨迹，一次性发布整条轨迹
                    rule_engine_->decide(agents_, dt);
                }
                realAdvance(dt);        //根据真实车身位置重新定位
                logAgentStatus();
                marker_pub_->publish(
                    agents_, visited_slots_, rule_engine_->conflicts(),
                    marker_pub_->hasSubscribers()
                        ? rule_engine_->conflictResourceMarkers(agents_)
                        : std::vector<forklift_planner::multi_vehicle::
                              ConflictMarker>{});
                publishRealTrailMarkers();
                return;
            }

        //3. 实车模式----滚动时域规划
            updateDwellAndTasks(dt);    //更新任务---任务   dt 
            rule_engine_->decide(agents_, dt);      //规则调度---决策
            realAdvance(dt);        //  用真实位姿更新车辆状态---执行
            if (tick_count_ % 5 == 0) runDeadlockRecovery();        //5*0.1=0.5s进行一次死锁检测
            if (force_horizon_refresh_ ||
                tick_count_ % rb_horizon_refresh_ == 0) {
                publishHorizon();   // 新航段立即发布；否则20*0.1=2s刷新
                force_horizon_refresh_ = false;
            }
            publishRealOutputs(dt);
            marker_pub_->publish(
                agents_, visited_slots_, rule_engine_->conflicts(),
                marker_pub_->hasSubscribers()
                    ? rule_engine_->conflictResourceMarkers(agents_)
                    : std::vector<forklift_planner::multi_vehicle::
                          ConflictMarker>{});
            publishRealTrailMarkers();
            return;
        }

        //=======仿真模式=======
        // dt = 1.0 / pp_.update_rate; dt= 1 / 10 = 0.1s。

        // 从当前仿真状态推进一拍。滚动模式下，普通协调只在构建10秒计划时
        // 运行；随后20拍(2秒)逐拍执行冻结计划。0.1秒层只保留任务事件、
        // 运动学推进和 advanceVehicles 内不可关闭的物理碰撞兜底。
        updateDwellAndTasks(dt);
        if (rb_one_shot_traj_) {
            rule_engine_->decide(agents_, dt);
        } else {
            if (simulationPlanNeedsRefresh()) {
                publishHorizon();
                force_horizon_refresh_ = false;
            }
            if (!executeSimulationPlanSample()) {
                // A task/path event may invalidate a just-built frame. Rebuild
                // once immediately; only fall back to a current decision if
                // planning itself produced no executable frame.
                publishHorizon();
                force_horizon_refresh_ = false;
                if (!executeSimulationPlanSample()) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[sim_plan] no executable frame; using one safe "
                        "current-step decision");
                    rule_engine_->decide(agents_, dt);
                }
            }
        }
        applyClusterAdmissionCounterfactual(dt);
        advanceVehicles(dt);
        diagnoseA1ExitIntrusions();


        //4. 仿真模式---一次性触发完成
        if (tick_count_ % 5 == 0) runDeadlockRecovery();

        if (rb_one_shot_traj_) {    
            if (!one_shot_published_) one_shot_published_ = publishFullTrajectories();
        }

        logAgentStatus();
        logStuckDiagnostics();
        marker_pub_->publish(
            agents_, visited_slots_, rule_engine_->conflicts(),
            marker_pub_->hasSubscribers()
                ? rule_engine_->conflictResourceMarkers(agents_)
                : std::vector<forklift_planner::multi_vehicle::
                      ConflictMarker>{});
    }
    
    //===========================================================================


    void publishSimTrackMarkers() {
        visualization_msgs::MarkerArray arr;
        for (size_t i = 0; i < agents_.size(); ++i) {
            const RoughPath& path = agents_[i].track.path();
            if (path.size() < 2) continue;
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id; m.header.stamp = ros::Time::now();
            m.ns = "sim_track"; m.id = static_cast<int>(i);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0; m.scale.x = 0.02;
            m.color = agents_[i].color; m.color.a = 0.9;
            for (const auto& p : path) {
                geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = 0.08;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    void publishRealTrailMarkers() {
        visualization_msgs::MarkerArray arr;
        const ros::Time now = ros::Time::now();
        for (size_t i = 0; i < real_trails_.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            if (real_trails_[i].size() < 2) continue;
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = now;
            m.ns = "real_trail";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.018;
            if (i < agents_.size()) {
                m.color = agents_[i].color;
            } else {
                m.color = rgba(1.0f, 1.0f, 1.0f, 1.0f);
            }
            m.color.a = 1.0;
            for (const auto& p : real_trails_[i]) m.points.push_back(p);
            arr.markers.push_back(m);
        }
        if (!arr.markers.empty()) horizon_marker_pub_.publish(arr);
    }

    // ───────── 实车模式:I/O 建立 + 摆位打印 ─────────
    void setupRealIO() {
        const int n = cfg_.vehicle_count;
        traj_pubs_.resize(n);
        speed_pubs_.resize(n);
        state_pubs_.resize(n);
        real_x_.assign(n, 0.0);
        real_y_.assign(n, 0.0);
        real_yaw_.assign(n, 0.0);
        real_pose_ok_.assign(n, false);
        rb_prev_path_s_.assign(n, 0.0);
        rb_cmd_speed_.assign(n, 0.0);
        rb_last_seen_.assign(n, 0.0);
        rb_published_gen_.assign(n, -1);
        one_shot_done_.assign(n, false);
        rb_track_gen_.assign(n, -1);
        rb_logged_gen_.assign(n, -1);
        real_trails_.assign(n, {});
        for (int i = 0; i < n; ++i) {
            traj_pubs_[i] = nh_.advertise<sandbox_msgs::Trajectory>(
                "/traj_" + std::to_string(i), 1, /*latch=*/true);
            speed_pubs_[i] = nh_.advertise<std_msgs::Float64>(
                "/coord_speed_" + std::to_string(i), 1, /*latch=*/false);
            state_pubs_[i] = nh_.advertise<std_msgs::String>(
                "/coord_state_" + std::to_string(i), 1, /*latch=*/false);
        }
        object_sub_ = nh_.subscribe("/object", 20,
                                    &MultiVehiclePatrolNode::objectCallback, this);
        // 启动/急停键(由独立键盘节点 estop_key.py 发):Enter→/rb_start 开跑;空格→/estop 切换急停。
        // 独立节点是因为 roslaunch 起的本节点拿不到终端 stdin,键盘要在自己的终端 tab 里读。
        start_sub_ = nh_.subscribe("/rb_start", 1,
                                   &MultiVehiclePatrolNode::rbStartCallback, this);
        estop_sub_ = nh_.subscribe("/estop", 1,
                                   &MultiVehiclePatrolNode::estopCallback, this);
        // AD 滚动时域参数:推演/发布时长 + 刷新周期(拍)。
    
        rb_horizon_ = cfg_.rolling_horizon;
        rb_horizon_refresh_period_ = cfg_.rolling_refresh_period;
        rb_horizon_refresh_ = std::max(1, (int)std::lround(rb_horizon_refresh_period_ * pp_.update_rate));
       
        ROS_WARN("[real] AD rolling horizon: horizon=%.1fs, refresh every %d ticks",
         rb_horizon_, rb_horizon_refresh_);
        // 一次性整条轨迹模式(默认开):start 后推演全程发一次 latch,对接原版 pure_pursuit 按时间跟踪。
        
        rb_one_shot_traj_ = cfg_.one_shot_traj;
        ros::param::param("~full_horizon", rb_full_horizon_, rb_full_horizon_);
        ROS_WARN("[real] 轨迹发布模式: %s (full_horizon=%.0fs)",
                 rb_one_shot_traj_ ? "一次性整条/纯盲跟" : "AD滚动时域", rb_full_horizon_);
        // 起始摆位标记:发到现有 marker topic(不同 ns),RViz 不改配置即可显示足迹+ID+朝向。
        start_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 1, /*latch=*/true);
        horizon_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/forklift_planner/markers", 10);   // 推演轨迹可视化(同topic不同ns,RViz直接显示)
        // 方案一:打印每辆车应摆放的真实坐标(track 起点)。
        ROS_WARN("==== 实车摆位(请把每辆车按编号摆到下列位置, 单位 m, yaw 弧度)====");
        std::ofstream ofs(realbridge_positions_file_);
        ofs << "# 实车摆位(按编号)。id slot x y yaw,单位 m/rad。\n";
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) { ROS_WARN("  车 %d: (无路径)", v.id); continue; }
            const auto p0 = v.track.poseAtS(0.0);
            ROS_WARN("  车 %d → 库位 %d, (x=%.3f, y=%.3f, yaw=%.3f)",
                     v.id, v.current_slot, p0.x, p0.y, p0.theta);
            char line[160];
            std::snprintf(line, sizeof(line), "%3d %4d %7.3f %7.3f %7.3f\n",
                          v.id, v.current_slot, p0.x, p0.y, p0.theta);
            ofs << line;
        }
        ofs.close();
        publishStartMarkers();  // RViz 画出每车起点:足迹框 + ID 数字 + 朝向箭头
    }

    // 起始摆位 RViz 标记:每车在 track 起点画 足迹框(LINE_STRIP)+ ID 文字 + 朝向箭头,
    // 各车独立颜色,latched 发到 /forklift_planner/markers(ns=start_*,与运行期 marker 不冲突)。
    void publishStartMarkers() {
        auto colorFor = [](int id) {
            static const float c[8][3] = {{1,0,0},{0,1,0},{0,0.5,1},{1,0.85,0},
                                          {1,0,1},{0,1,1},{1,0.5,0},{0.7,0.7,0.7}};
            std_msgs::ColorRGBA col; col.a = 1.0;
            col.r = c[id % 8][0]; col.g = c[id % 8][1]; col.b = c[id % 8][2];
            return col;
        };
        visualization_msgs::MarkerArray arr;
        for (const VehicleAgent& v : agents_) {
            if (v.track.empty()) continue;
            const RoughWp p0 = v.track.poseAtS(0.0);
            // 车身足迹:用半透明 CUBE(车身几何中心 + 朝向 + 车长×车宽),抬到 z 上方避免被
            // 地图遮住(之前 LINE_STRIP 太细且与地图同平面 → 看不见)。半透明可对位。
            const RoughWp bc = forklift_planner::multi_vehicle::bodyCenterPose(p0, mp_);
            visualization_msgs::Marker box;
            box.header.frame_id = pp_.frame_id;
            box.header.stamp = ros::Time::now();
            box.ns = "start_footprint"; box.id = v.id;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.position.x = bc.x; box.pose.position.y = bc.y; box.pose.position.z = 0.02;
            box.pose.orientation.z = std::sin(bc.theta / 2.0);
            box.pose.orientation.w = std::cos(bc.theta / 2.0);
            box.scale.x = mp_.vehicle_length; box.scale.y = mp_.vehicle_width; box.scale.z = 0.03;
            box.color = colorFor(v.id); box.color.a = 0.35;  // 半透明
            arr.markers.push_back(box);
            // ID 文字
            visualization_msgs::Marker txt;
            txt.header = box.header;
            txt.ns = "start_id"; txt.id = v.id;
            txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::Marker::ADD;
            txt.pose.position.x = p0.x; txt.pose.position.y = p0.y; txt.pose.position.z = 0.12;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.10;
            txt.color = colorFor(v.id);
            txt.text = std::to_string(v.id);
            arr.markers.push_back(txt);
            // 朝向箭头:从车身中心沿车头方向画一条明显的箭头(抬到车身上方 z=0.06,
            // ARROW 用两点时 scale.x=杆径/scale.y=箭头径/scale.z=箭头长——之前 scale.z=0 没箭头)。
            visualization_msgs::Marker ar;
            ar.header = box.header;
            ar.ns = "start_heading"; ar.id = v.id;
            ar.type = visualization_msgs::Marker::ARROW;
            ar.action = visualization_msgs::Marker::ADD;
            geometry_msgs::Point a0, a1;
            a0.x = bc.x; a0.y = bc.y; a0.z = 0.06;
            a1.x = bc.x + 0.18 * std::cos(p0.theta);
            a1.y = bc.y + 0.18 * std::sin(p0.theta); a1.z = 0.06;
            ar.points.push_back(a0); ar.points.push_back(a1);
            ar.scale.x = 0.02; ar.scale.y = 0.045; ar.scale.z = 0.05;
            ar.color = colorFor(v.id); ar.color.a = 1.0;
            ar.pose.orientation.w = 1.0;
            arr.markers.push_back(ar);
        }
        // 地图原点+XY轴一并经 latched 话题先发(real_mode tick 要等所有车动捕就绪才publish,
        // 摆车前看不到轴 → 在这儿先发,标定时立刻可见)。
        marker_pub_->addOriginAxes(arr);
        start_marker_pub_.publish(arr);
        ROS_WARN("[multi_patrol] 已在 RViz 画出 8 车起始摆位(足迹+ID+朝向)+地图原点XY轴,对着摆即可。");
        ROS_WARN("[multi_patrol] 摆好后在【键盘节点终端】按 Enter 开跑;运行中按 空格 = 急停(再按解除)。");
    }

    // /object(动捕,mm,后轮中心=后轴参考)→ 各车真实位姿(转米)
    void objectCallback(const sandbox_msgs::AprilObject::ConstPtr& msg) {
        if (msg->type != sandbox_msgs::AprilObject::VEHICLE) return;
        const int id = msg->id;
        if (id < 0 || id >= static_cast<int>(agents_.size())) return;
        real_x_[id] = msg->x / 1000.0;  // mm→m(nokov 发 mm,pure_pursuit 也 /1000)
        real_y_[id] = msg->y / 1000.0;
        real_yaw_[id] = msg->yaw;
        real_pose_ok_[id] = true;
        rb_last_seen_[id] = ros::Time::now().toSec();  // 动捕看门狗:记最后一次见到的时刻
        // RViz 显示真实位姿(实际位置,非投影):同步进 agent 供 marker 用。
        agents_[id].real_pose_valid = true;
        agents_[id].real_x = real_x_[id];
        agents_[id].real_y = real_y_[id];
        agents_[id].real_yaw = real_yaw_[id];

        if (targetEnabled(id) && id < static_cast<int>(real_trails_.size())) {
            geometry_msgs::Point p;
            p.x = real_x_[id];
            p.y = real_y_[id];
            p.z = 0.12;
            auto& trail = real_trails_[id];
            const bool moved_enough =
                trail.empty() || std::hypot(p.x - trail.back().x, p.y - trail.back().y) > 0.01;
            if (moved_enough) {
                trail.push_back(p);
                constexpr size_t kMaxRealTrailPoints = 2000;
                while (trail.size() > kMaxRealTrailPoints) trail.pop_front();
            }
        }
    }

    // Enter:摆位完成、开跑。未全部就位也允许启动(未就位的车由动捕看门狗摁停),但会告警。
    void rbStartCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data || rb_started_) return;
        int missing = 0;
        for (size_t i = 0; i < real_pose_ok_.size(); ++i) {
            if (targetEnabled(static_cast<int>(i)) && !real_pose_ok_[i]) ++missing;
        }
        rb_started_ = true;
        if (missing > 0)
            ROS_WARN("[real] 收到启动,但还有 %d 辆车动捕未就位 → 它们会被看门狗摁停,直到被看到。", missing);
        ROS_WARN("[real] *** 已启动:开始协调推进 ***");
    }

    // 空格:急停切换。true=全车瞬时停;再按一下=false 恢复协调速度。
    void estopCallback(const std_msgs::Bool::ConstPtr& msg) {
        rb_estop_ = msg->data;
        ROS_ERROR("[real] *** 急停 %s ***", rb_estop_ ? "已触发(全车停)" : "已解除(恢复)");
    }

    // 摆位阶段节流播报:哪些车动捕已到位 / 还缺哪些(否则启动门控是"哑"的,现场不知在等谁)。
    void logPlacementStatus() {
        std::string seen, miss;
        for (size_t i = 0; i < real_pose_ok_.size(); ++i) {
            if (!targetEnabled(static_cast<int>(i))) continue;
            (real_pose_ok_[i] ? seen : miss) += "V" + std::to_string(i) + " ";
        }
        if (miss.empty())
            ROS_WARN_THROTTLE(2.0, "[real] 全部就位 ✓ [%s] —— 去【启动/急停键盘】终端按 Enter 启动"
                              "(在打印本日志的终端里按 Enter 无效!)", seen.c_str());
        else
            ROS_WARN_THROTTLE(2.0, "[real] 摆位中:已到位 [%s] 还缺 [%s](等动捕看到)。"
                              "齐了去【启动/急停键盘】终端按 Enter。", seen.c_str(), miss.c_str());
    }

    static double projectOntoTrackRange(const forklift_planner::multi_vehicle::PathTrack& tr,
                                        double x, double y, double lo, double hi) {
        double best_s = lo, best_d2 = 1e18;
        for (double s = lo; s <= hi + 1e-9; s += 0.01) {
            const auto p = tr.poseAtS(s);
            const double d2 = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
            if (d2 < best_d2) { best_d2 = d2; best_s = s; }
        }
        return best_s;
    }

    // 替代 advanceVehicles 的「位置推进」:实车位置取自 /object 投影,不做 sim 积分/硬护栏。
    // 到库→DWELL 逐字节复刻 advanceVehicles 705-714,保证和 sim 同样的"到点停 10s"。
    void realAdvance(double dt) {
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            if (v.mode != VehicleMode::ACTIVE || v.track.empty()) continue;
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;  // 动捕还没看到这辆 → 别拿 (0,0) 投影出垃圾 path_s
            if (rb_track_gen_[i] != v.path_gen) {  // 新任务/新路径 → path_s 归零
                rb_track_gen_[i] = v.path_gen;
                rb_prev_path_s_[i] = 0.0;
            }
            const double lo = std::max(0.0, rb_prev_path_s_[i] - 0.10);
            const double hi = std::min(v.track.length(), rb_prev_path_s_[i] + 0.50);
            double new_s = projectOntoTrackRange(v.track, real_x_[i], real_y_[i], lo, hi);
            new_s = std::max(new_s, rb_prev_path_s_[i]);  // 单调不减(沿固定路径只前进)
            v.current_speed = std::max(0.0, std::min((new_s - rb_prev_path_s_[i]) / dt,
                                                     cfg_.max_speed));
            v.path_s = new_s;
            rb_prev_path_s_[i] = new_s;
            // 到库→DWELL —— 复刻 advanceVehicles 705-714(实车容差 cfg_.real_arrive_tol)
            if (v.path_s >= v.track.length() - cfg_.real_arrive_tol) {
                // 关键(联动 rule_engine):sim 里 DWELL 车 path_s≈length,故其碰撞足迹
                // poseAtS(path_s)=槽位;rule_engine 73/85/737/742 处直接用 poseAtS(path_s)
                // 算静止车足迹(没套 DWELL?length:path_s)。实车若停在 length-5cm,这些足迹
                // 就比 sim 偏 5cm → 与停驻车的冲突判定偏离仿真。故到点即把 path_s 夹到 length,
                // 让"协调眼里的 DWELL 车"=精确槽位,与 sim 逐字节一致(实际 5cm 物理差归控制器管)。
                v.path_s = v.track.length();
                rb_prev_path_s_[i] = v.track.length();
                handleLegArrival(v);
            }
        }
    }

    // 实车硬护栏(第二层兜底,替代 sim 里 advanceVehicles 的 hard_collision_guard):
    // 用真实 /object 位姿算两两足迹(充气 real_emergency_margin),重叠即双方急停。
    // 预测层(decide)是第一层;它漏判时这层兜住,防真车相撞。返回每车是否需急停。
    // 设计取舍:margin<预测层间距→正常不触;只在两车逼近到 <margin 才停(双停=安全优先,
    // 死活留给协调/死锁检测理顺)。DWELL/idle 车也算静态障碍(用其真实位姿)。
    std::vector<bool> realHardGuard() {
        std::vector<bool> estop(agents_.size(), false);
        const double m = cfg_.real_emergency_margin;
        if (m <= 0.0) return estop;  // 0 = 关闭
        auto realBody = [&](size_t i) {
            RoughWp p; p.x = real_x_[i]; p.y = real_y_[i];
            p.theta = real_yaw_[i]; p.type = WpType::FORWARD;
            return forklift_planner::multi_vehicle::makeBody(p, mp_, m);
        };
        for (size_t i = 0; i < agents_.size(); ++i) {
            if (cfg_.real_mode && !real_pose_ok_[i]) continue;
            const auto bi = realBody(i);
            for (size_t j = i + 1; j < agents_.size(); ++j) {
                if (!real_pose_ok_[j]) continue;
                if (forklift_planner::multi_vehicle::overlaps(bi, realBody(j))) {
                    // 只急停"在动的"那辆(ACTIVE);停驻/空闲车本就静止,标它无意义且会污染状态。
                    // 静态车是障碍方,移动车才是要被拦下的一方(若两辆都在动则都停)。
                    bool any = false;
                    if (agents_[i].mode == VehicleMode::ACTIVE) { estop[i] = true; any = true; }
                    if (agents_[j].mode == VehicleMode::ACTIVE) { estop[j] = true; any = true; }
                    if (any) {
                        ROS_ERROR_THROTTLE(0.5, "[real] 硬护栏急停: V%zu 与 V%zu 实测足迹逼近"
                                           "(<%.2fm)。预测层疑似漏判,查 logger 车间距/coord_flag。",
                                           i, j, m);
                    }
                }
            }
        }
        return estop;
    }

    // 曲率限速(规划侧运动学):弯道允许速度 v≤√(a_lat/κ)。用 path_s 附近三点位置估 κ(Menger,
    // 对动捕/cusp 比航向差稳);限到 [creep, +∞)避免曲率尖点把车停死。返回该车此刻的速度上限。
    double curvatureSpeed(const VehicleAgent& v) const {
        if (cfg_.lat_accel_max <= 0.0 || v.track.empty()) return 1e9;  // 关闭
        const double L = v.track.length(), ds = 0.05;
        const double s = std::min(std::max(v.path_s, 0.0), L);
        const auto A = v.track.poseAtS(std::max(0.0, s - ds));
        const auto B = v.track.poseAtS(s);
        const auto C = v.track.poseAtS(std::min(L, s + ds));
        const double abx=B.x-A.x, aby=B.y-A.y, acx=C.x-A.x, acy=C.y-A.y;
        const double lab=std::hypot(abx,aby), lbc=std::hypot(C.x-B.x,C.y-B.y), lac=std::hypot(acx,acy);
        if (lab<1e-4||lbc<1e-4||lac<1e-4) return 1e9;
        const double kappa = 2.0*std::fabs(abx*acy - aby*acx)/(lab*lbc*lac);  // Menger κ
        if (kappa < 1e-3) return 1e9;
        const double vc = std::sqrt(cfg_.lat_accel_max / kappa);
        const double v_floor = cfg_.nominal_speed * cfg_.creep_ratio;          // 弯再急也不低于 creep,不停死
        return std::max(vc, v_floor);
    }

    // 发 /traj_i(路径,换任务才重发) + /coord_speed_i(带符号实时速度,每拍)
    void publishRealOutputs(double dt) {
        const double now = ros::Time::now().toSec();
        const std::vector<bool> estop = realHardGuard();  // 第二层:真实足迹逼近→急停
        for (size_t i = 0; i < agents_.size(); ++i) {
            VehicleAgent& v = agents_[i];
            // 注:几何 /traj 已由 publishHorizon(滚动时域时间参数化轨迹)发布,这里不再发 /traj。
            // 速度幅值=协调动作档,再被曲率限速卡住(规划侧运动学:弯道降速)。方向=路径段(倒车负)。STOP→0。
            double mag = rule_engine_->speedForAction(v.action);
            mag = std::min(mag, curvatureSpeed(v));   // 曲率限速(lat_accel_max,0=关)
            // 方向:当前段倒车,或【前方一小段即将进入倒车段】→ 负(倒车)。后者关键:realAdvance 的
            // path_s 单调只增,前进逼近 FORWARD→REVERSE 的 cusp 时,path_s 越不过 cusp(前进会冲偏、
            // 投影卡在 cusp)→ 若只看 typeAtS(path_s) 永远 FORWARD → 车冲过该倒车处不倒(sim 积分不暴露)。
            // 故前瞻 0.10m:逼近 cusp 即提前给负速度,车减速→cusp 停→倒入倒车段→path_s 越过,死锁解开。
            double dir = 1.0;
            if (!v.track.empty()) {
                const double look_ahead = 0.10;  // m
                const bool rev = v.track.typeAtS(v.path_s) == WpType::REVERSE ||
                                 v.track.typeAtS(std::min(v.path_s + look_ahead, v.track.length()))
                                     == WpType::REVERSE;
                if (rev) dir = -1.0;
            }
            double target = dir * mag;
            // 动捕看门狗:该车位姿失联(>0.5s)→ 强制目标速度 0。否则控制器拿陈旧位姿
            // 盲走、底盘又无超时 → 跑飞。只压这一辆的输出速度,协调逻辑/其它车不受影响
            //(协调端看它 path_s 不动,自会让旁车等它,安全)。
            // real_pose_timeout ≤ 0 → 关闭动捕失联停(与 emerg_margin=0、cmd_timeout≤0 统一约定)
            const bool stale = cfg_.real_pose_timeout > 0.0 &&
                               (now - rb_last_seen_[i]) > cfg_.real_pose_timeout;
            const char* flag = "OK";
            bool instant_zero = false;
            if (rb_estop_) {          // 操作员空格急停:最高优先级,瞬时 0(不走斜坡,要快)
                target = 0.0; instant_zero = true; flag = "EKEY";
            } else if (estop[i]) {    // 第二层硬护栏:真实足迹逼近 → 急停
                target = 0.0;
                flag = "ESTOP";
            } else if (stale) {       // 动捕失联 → 强制 0,防控制器拿陈旧位姿盲走跑飞
                target = 0.0;
                flag = "STALE";
                ROS_WARN_THROTTLE(1.0, "[real] 车 %zu 动捕失联 %.1fs → 强制STOP",
                                  i, now - rb_last_seen_[i]);
            }
            // 加速度限制(与 sim 一致):按 max_accel/max_decel 斜坡逼近目标,而非阶跃。
            // 这样真车速度剖面 = 仿真,停在协调按 max_decel 算的停止线处;cusp 处平滑过 0 反向。
            // 注:硬护栏/STALE 走斜坡降速(避免甩动,距离近时本就低速);操作员急停 EKEY 例外,瞬时归 0。
            if (instant_zero) rb_cmd_speed_[i] = 0.0;
            else rb_cmd_speed_[i] = limitedSpeed(rb_cmd_speed_[i], target, dt);
            std_msgs::Float64 sp;
            sp.data = rb_cmd_speed_[i];
            speed_pubs_[i].publish(sp);

            // 只读协调状态。格式 "mode,action,blk,brk,wait,slot->tgt,flag,seg,s/len" 供 logger 落列。
            // seg=当前路径段方向(FWD/REV,来自 typeAtS(path_s));s/len=路径进度——排查"该倒车却前进"。
            const char* seg = (dir < 0.0) ? "REV" : "FWD";
            const double len = v.track.empty() ? 0.0 : v.track.length();
            char buf[140];
            std::snprintf(buf, sizeof(buf), "%s,%s,%d,%d,%.1f,%d->%d,%s,%s,%.2f/%.2f",
                          modeName(v.mode), actionName(v.action), v.blocker_id,
                          v.deadlock_breaker ? 1 : 0, v.wait_time,
                          v.current_slot, v.target_slot, flag, seg, v.path_s, len);
            std_msgs::String st;
            st.data = buf;
            state_pubs_[i].publish(st);
            // 在动时节流打印:进度+段向+协调速度,实时看"倒车段是否被识别、coord_speed 是否变负"。
            if (v.mode == VehicleMode::ACTIVE)
                ROS_INFO_THROTTLE(1.0, "[real] V%zu s=%.2f/%.2f seg=%s coord_speed=%+.2f action=%s",
                                  i, v.path_s, len, seg, rb_cmd_speed_[i], actionName(v.action));
        }
    }

    void publishTraj(int id, const forklift_planner::multi_vehicle::PathTrack& tr) {
        sandbox_msgs::Trajectory msg;
        msg.target = id;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        const double len = tr.length();
        std::string segs; bool prev_rev = false; double seg_start = 0.0;  // 诊断:倒车段 s 区间
        for (double s = 0.0; s <= len + 1e-9; s += 0.02) {
            const double ss = std::min(s, len);
            const auto p = tr.poseAtS(ss);
            const bool rev = (tr.typeAtS(ss) == WpType::REVERSE);
            sandbox_msgs::TrajectoryPoint tp;
            tp.x = p.x; tp.y = p.y; tp.yaw = p.theta;
            tp.velocity = rev ? -1.0 : 1.0;  // 方向标记;速度幅值走 /coord_speed_i
            tp.time = 0.0;
            msg.points.push_back(tp);
            if (rev && !prev_rev) seg_start = ss;
            if (!rev && prev_rev) segs += "[" + std::to_string(seg_start).substr(0,4) + "," +
                                            std::to_string(ss).substr(0,4) + "] ";
            prev_rev = rev;
        }
        if (prev_rev) segs += "[" + std::to_string(seg_start).substr(0,4) + "," +
                                  std::to_string(len).substr(0,4) + "]";
        ROS_WARN("[real] V%d 新路径 len=%.2f  倒车段 s=%s", id, len,
                 segs.empty() ? "无(全程前进)" : segs.c_str());
        traj_pubs_[id].publish(msg);
    }

    ros::NodeHandle nh_;
    ros::Timer timer_;

    MapParam mp_;
    PlannerParam pp_;
    forklift_planner::multi_vehicle::MultiVehicleConfig cfg_;

    std::unique_ptr<ForkliftMap> map_;
    std::unique_ptr<PathGenerator> generator_;
    std::unique_ptr<forklift_planner::multi_vehicle::TaskAllocator> allocator_;
    std::unique_ptr<forklift_planner::multi_vehicle::RuleEngine> rule_engine_;
    std::unique_ptr<forklift_planner::multi_vehicle::FutureMissionPlanBuilder>
        future_mission_plan_builder_;
    std::unique_ptr<forklift_planner::multi_vehicle::FutureTrajectoryGenerator>
        future_trajectory_generator_;
    std::unique_ptr<
        forklift_planner::multi_vehicle::LegacyPredictionShadowGenerator>
        legacy_prediction_shadow_generator_;
    std::unique_ptr<forklift_planner::multi_vehicle::PredictionShadowComparator>
        prediction_shadow_comparator_;
    std::unique_ptr<
        forklift_planner::multi_vehicle::TimedConflictShadowChecker>
        timed_conflict_shadow_checker_;
    std::unique_ptr<
        forklift_planner::multi_vehicle::FutureConflictZoneShadowBuilder>
        future_conflict_zone_shadow_builder_;
    std::unique_ptr<
        forklift_planner::multi_vehicle::FutureConflictClusterShadowBuilder>
        future_conflict_cluster_shadow_builder_;
    std::unique_ptr<
        forklift_planner::multi_vehicle::FutureClusterArbitrationShadow>
        future_cluster_arbitration_shadow_;
    std::unique_ptr<
        forklift_planner::multi_vehicle::FutureClusterAdmissionShadow>
        future_cluster_admission_shadow_;
    std::unique_ptr<forklift_planner::multi_vehicle::
        FutureClusterAdmissionShadowTracker>
        future_cluster_admission_shadow_tracker_;
    std::unique_ptr<forklift_planner::multi_vehicle::
        ClusterAdmissionEvaluator>
        cluster_admission_evaluator_;
    std::unique_ptr<forklift_planner::multi_vehicle::
        ClusterAdmissionCounterfactualSimulator>
        cluster_admission_counterfactual_simulator_;
    bool one_shot_ = false;  // false: continuously execute B->A1->B transports
    std::unique_ptr<forklift_planner::multi_vehicle::MarkerPublisher> marker_pub_;
    std::unique_ptr<forklift_planner::multi_vehicle::TrafficResourceMap> resource_map_;
    std::vector<VehicleAgent> agents_;
    std::vector<bool> visited_slots_;
    int target_only_ = -1;  // realbridge debug: -1 = all vehicles, otherwise control only this id
    std::string debug_log_dir_;
    std::string coord_log_file_;
    std::string onset_log_file_;
    std::string realbridge_positions_file_;
    std::ofstream coord_log_;
    std::string coord_log_source_ = "REAL";
    uint64_t coord_log_plan_id_ = 0;
    int coord_log_frame_id_ = -1;
    int coord_log_rollout_step_ = -1;
    bool coord_log_suppressed_ = false;
    uint64_t rollout_log_id_ = 0;

    // ── 实车模式(real_mode)I/O ──────────────────────────────────────────────
    ros::Subscriber object_sub_;                       // /object 动捕位姿(mm)
    ros::Publisher start_marker_pub_;                  // 起始摆位标记(足迹框+ID+朝向,latched)
    std::vector<ros::Publisher> traj_pubs_, speed_pubs_;// /traj_i(路径) + /coord_speed_i(实时速度)
    std::vector<ros::Publisher> state_pubs_;            // /coord_state_i(只读调试:停车原因)
    ros::Publisher horizon_marker_pub_;                 // 推演 5s 轨迹的 RViz 可视化(LINE_STRIP/车)
    ros::Subscriber start_sub_, estop_sub_;             // /rb_start(Enter开跑) /estop(空格急停切换)
    bool rb_started_ = false;                           // 摆位完成、按Enter后才推进
    bool rb_estop_ = false;                             // 操作员急停:true=全车瞬时停
    bool rb_estop_prev_ = false;                        // 上一拍急停态(检测按下/解除边沿,one_shot急停用)
    
    double rb_horizon_ = 10.0;                           // AD 滚动时域:每次推演/发布的未来时长(s)
    double rb_horizon_refresh_period_ = 2.0;
    int rb_horizon_refresh_ = 20;                        // 每多少拍重新推演刷新一次(5拍=0.5s)
    bool rb_one_shot_traj_ = false;                      // 一次性整条轨迹模式(默认):start后推演全程发一次latch
    bool force_horizon_refresh_ = false;                 // 新航段安装后立即覆盖发布

    // 仿真专用:当前10秒协调计划及其2秒执行窗口。real_mode分支不读取这些字段。
    std::vector<SimPlanFrame> sim_plan_frames_;
    size_t sim_plan_cursor_ = 0;
    bool sim_plan_valid_ = false;
    uint64_t sim_plan_id_ = 0;
    double sim_plan_start_time_ = 0.0;
    forklift_planner::multi_vehicle::RuleEngine::FutureA1Commitment
        future_a1_commitment_;
    std::vector<FutureMissionTrajectory> future_trajectory_cache_;
    std::vector<PredictionShadowReport> prediction_shadow_reports_;
    std::vector<std::vector<
        forklift_planner::multi_vehicle::LegacyPredictionSample>>
        legacy_prediction_cache_;
    std::vector<TimedConflictShadowReport>
        timed_conflict_shadow_reports_;
    std::vector<FutureConflictZone> future_conflict_zones_;
    std::vector<FutureConflictCluster> future_conflict_clusters_;
    std::vector<ClusterReservationShadow> cluster_arbitration_shadows_;
    std::vector<ClusterAdmissionShadow> cluster_admission_shadows_;
    std::vector<ClusterAdmissionConstraint>
        cluster_admission_constraints_;
    std::vector<ShadowVehicleState> counterfactual_shadow_states_;
    unsigned long long cluster_admission_evaluations_ = 0;
    unsigned long long cluster_admission_valid_count_ = 0;
    unsigned long long cluster_admission_prevent_count_ = 0;
    unsigned long long cluster_admission_late_mixing_count_ = 0;
    unsigned long long dynamic_admission_feasible_count_ = 0;
    unsigned long long dynamic_admission_infeasible_count_ = 0;
    bool cluster_counterfactual_shadow_enabled_ = false;
    unsigned long long counterfactual_stop_vehicle_ticks_ = 0;
    unsigned long long counterfactual_go_overrides_ = 0;
    unsigned long long counterfactual_cluster_releases_ = 0;
    unsigned long long counterfactual_waiter_resumes_ = 0;
    unsigned long long counterfactual_waiter_entry_violations_ = 0;
    unsigned long long counterfactual_late_braking_events_ = 0;
    unsigned long long dynamic_admission_stop_requests_ = 0;
    unsigned long long dynamic_admission_action_changes_ = 0;
    double counterfactual_max_cluster_wait_ = 0.0;
    std::set<std::tuple<int, int, int, int, long long>>
        dynamic_admission_decision_log_keys_;
    std::set<std::tuple<int, int, int, int, int, int, long long, long long>>
        future_conflict_zone_log_keys_;
    std::vector<std::pair<std::string, int>> future_mission_marker_keys_;
    std::vector<std::pair<std::string, int>>
        future_shadow_conflict_marker_keys_;
    std::vector<std::pair<std::string, int>>
        future_conflict_zone_marker_keys_;
    std::vector<std::pair<std::string, int>>
        future_conflict_cluster_marker_keys_;
    std::vector<std::pair<std::string, int>>
        future_cluster_arbitration_marker_keys_;
    std::vector<std::pair<std::string, int>>
        future_cluster_admission_marker_keys_;
    std::vector<std::pair<std::string, int>>
        future_cluster_counterfactual_marker_keys_;
    
    double rb_full_horizon_ = 180.0;                    // 一次性模式:全程推演上限时长(s),尾部静止点会裁掉
    bool one_shot_published_ = false;                   // 一次性轨迹是否已全部发完(防重复发)
    std::vector<bool> one_shot_done_;                   // 各车一次性轨迹是否已发(动捕晚到的车就位后补发)
    std::vector<int> rb_logged_gen_;                    // 已打印倒车段诊断的 path_gen(每车,防重复刷屏)
    std::vector<double> real_x_, real_y_, real_yaw_;   // 各车真实位姿(已 /1000 转米)
    std::vector<std::deque<geometry_msgs::Point>> real_trails_; // RViz 实车走过的真实轨迹(ns=real_trail)
    std::vector<bool> real_pose_ok_;                   // 动捕是否已收到该车
    std::vector<double> rb_prev_path_s_;               // 上一拍 path_s(局部投影+差分估速度)
    std::vector<double> rb_cmd_speed_;                 // 上一拍发出的速度命令(带符号),斜坡限速用
    std::vector<double> rb_last_seen_;                 // 各车动捕最后到达时刻(s),看门狗用
    // 动捕失联超时、到点容差 → 已提为 ROS 参数 cfg_.real_pose_timeout / real_arrive_tol,
    // 现场可不重编译直接调。到点容差需 > PP 的 2cm 硬停容差,否则 PP 停在 2cm 短处而 path_s
    // 到不了 length → 永不 DWELL → 不触发下一个任务(卡死);默认 5cm。
    std::vector<int> rb_published_gen_, rb_track_gen_; // 已发布的 path_gen / 上次见到的 path_gen
    std::vector<VehicleMode> last_logged_mode_;
    std::vector<MissionPhase> last_logged_mission_phase_;
    std::vector<VehicleAction> last_logged_action_;
    std::vector<std::string> last_logged_reason_;
    std::vector<int> last_logged_blocker_;
    std::vector<int> last_logged_task_count_;
    std::vector<ros::Time> last_status_log_time_;
    std::vector<ros::Time> last_diag_time_;  // TEMPORARY: [DIAG stuck] throttle
    unsigned long long tick_count_ = 0;
    std::set<std::pair<int, int>> active_a1_exit_intrusions_;
    double sim_time_ = 0.0;

    // Phase 6.2 diagnostic counters only; none feeds vehicle decisions.
    unsigned long long batch_stop_vehicle_ticks_ = 0;
    unsigned long long batch_wait_samples_ = 0;
    double batch_wait_sum_ = 0.0;
    unsigned long long batch_first_wedge_count_ = 0;
    unsigned long long batch_deadlock_episodes_ = 0;
    bool batch_deadlock_episode_active_ = false;
    double batch_max_observed_wait_ = 0.0;
    int batch_max_observed_wait_vehicle_ = -1;

    // 无头批处理(快速回归)统计。确定性仿真:同种子同代码必得同结果,故脱离 RViz、
    // 不按实时狂跑 N 拍即可在几秒内覆盖数小时仿真,直接数碰撞。
    unsigned long long hard_guard_events_ = 0;        // 硬护栏触发(碰撞)累计次数
    unsigned long long first_guard_tick_ = 0;         // 首次碰撞所在 tick(0=从未)
    std::set<std::pair<int, int>> hard_guard_pairs_;  // 涉及碰撞的车对
    unsigned long long deadlock_ticks_ = 0;           // 检测到持续死锁环的拍数累计
    unsigned long long deadlock_recoveries_ = 0;      // 成功重规划脱困次数
    bool sim_mode_ = false;                           // 前瞻仿真中:屏蔽计数/日志副作用
    std::map<int, int> predict_holds_;                // 车id→连续"预测性错峰扣车"次数(防极端饥饿)
    bool deadlock_logged_ = false;                    // 首次死锁是否已详打
    std::set<std::set<int>> dumped_clusters_;         // 已详打过的死锁簇(按成员集去重,编目用)
    std::map<int, double> last_replan_t_;             // 车id→上次重规划 sim_t(冷却用)

public:
    // 一辆车的紧凑状态行(诊断用,信息尽量全)。
    std::string vehLine(const VehicleAgent& v) const {
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "  V%d mode=%d act=%d reason=%s blk=%d brkr=%d task=%d slot=%d->%d "
                 "s=%.3f/%.3f rem=%.3f spd=%.3f wait=%.1f gen=%d",
                 v.id, (int)v.mode, (int)v.action, v.reason.c_str(), v.blocker_id,
                 (int)v.deadlock_breaker, v.task_count, v.current_slot,
                 v.target_slot, v.path_s, v.track.length(), v.remainingS(),
                 v.current_speed, v.wait_time, v.path_gen);
        return buf;
    }
    std::string fleetSnapshot() const {
        std::string s = "tick=" + std::to_string(tick_count_);
        for (const VehicleAgent& v : agents_) s += "\n" + vehLine(v);
        return s;
    }
    VehicleAgent* agentById(int id) {
        for (VehicleAgent& v : agents_)
            if (v.id == id) return &v;
        return nullptr;
    }
    // 打印某对车的冲突几何(委托 RuleEngine,拿到 se/sx/same_dir/committed/owner/following)。
    void dumpPair(int ia, int ib) {
        VehicleAgent* a = agentById(ia);
        VehicleAgent* b = agentById(ib);
        if (a && b) rule_engine_->debugDumpConflict(*a, *b);
    }

    // 死锁看门狗(C-第1步):跟 blocker_id 等待图找一个「持续死锁环」——环内每辆都
    // ACTIVE+STOP 且 wait_time≥min_wait(确为持续、非瞬时)。返回环成员 id(按链序),无则空。
    std::vector<int> findDeadlockCycle(double min_wait) const {
        auto idxOf = [&](int id) -> int {
            for (size_t i = 0; i < agents_.size(); ++i)
                if (agents_[i].id == id) return static_cast<int>(i);
            return -1;
        };
        for (const VehicleAgent& v0 : agents_) {
            if (v0.mode != VehicleMode::ACTIVE) continue;
            std::vector<int> path;
            int cur = v0.id;
            while (cur >= 0) {
                auto it = std::find(path.begin(), path.end(), cur);
                if (it != path.end()) {  // 链绕回 → 环 = [it, end)
                    std::vector<int> cyc(it, path.end());
                    bool all_stuck = cyc.size() >= 2;
                    for (int cid : cyc) {
                        int k = idxOf(cid);
                        if (k < 0 || agents_[k].wait_time < min_wait) { all_stuck = false; break; }
                    }
                    return all_stuck ? cyc : std::vector<int>{};
                }
                path.push_back(cur);
                int k = idxOf(cur);
                if (k < 0 || agents_[k].mode != VehicleMode::ACTIVE ||
                    agents_[k].action != VehicleAction::STOP)
                    break;
                cur = agents_[k].blocker_id;
            }
        }
        return {};
    }

    // 死锁恢复(C):检测「所有」持续环的成员 → 选其中等待最久(最该救)且不在冷却期的车,
    // 从当前位姿重规划到空库位脱困。不倒车、不强推。冷却防止反复重规划同一辆(churn)。
    void runDeadlockRecovery() {
        constexpr double kDeadlockWait = 25.0;  // 持续卡 >此秒数才算真死锁
        constexpr double kCooldown = 8.0;       // 同一辆两次重规划的最小间隔(给它时间驶离)
        const std::set<int> members = findDeadlockMembers(kDeadlockWait);
        if (members.empty()) return;
        ++deadlock_ticks_;
        // 选等待最久、且不在冷却期的成员当受害车。
        int victim = -1;
        double worst_wait = -1.0;
        for (int id : members) {
            const VehicleAgent* a = agentById_c(id);
            if (!a) continue;
            auto it = last_replan_t_.find(id);
            if (it != last_replan_t_.end() && sim_time_ - it->second < kCooldown) continue;
            if (a->wait_time > worst_wait) { worst_wait = a->wait_time; victim = id; }
        }
        if (victim < 0) return;  // 全在冷却 → 等下一拍
        if (!deadlock_logged_) {
            deadlock_logged_ = true;
            std::string ms;
            for (int id : members) ms += "V" + std::to_string(id) + " ";
            ROS_ERROR("[DEADLOCK] @tick=%llu sim_t=%.1fs 环成员=[%s] 首个受害车=V%d",
                      tick_count_, sim_time_, ms.c_str(), victim);
        }
        // 编目:每种不同的死锁簇(按成员集去重)各全面 dump 一次,catalog 出"到底几种特例"。
        if (dumped_clusters_.size() < 15 && dumped_clusters_.insert(members).second) {
            ROS_ERROR("[CLUSTER-CATALOG] 第 %zu 种 @tick=%llu sim_t=%.1fs",
                      dumped_clusters_.size(), tick_count_, sim_time_);
            dumpDeadlockCluster(members);
        }
        // 重规划脱困整套停用(enable_deadlock_recovery=false)。看门狗保留
        // 为纯检测(上方 deadlock_ticks 计数 + 首环日志),不再执行任何重规划/脱困动作。
        if (!cfg_.enable_deadlock_recovery) return;
        VehicleAgent* v = agentById(victim);
        if (!v) return;
        // 冷却按「尝试」记,而非仅「成功」记:换货位失败(当时清不出逃逸位)时,也冷却 8s 再试,
        // 别每 5 拍就对一堆候选位重算路径(实测那是 24.8 万条 clothoid 警告 + 大量 CPU 空转的根因)。
        last_replan_t_[victim] = sim_time_;
        if (allocator_->replanFromPose(*v, agents_)) {
            ++deadlock_recoveries_;
        }
    }

    const VehicleAgent* agentById_c(int id) const {
        for (const VehicleAgent& v : agents_) if (v.id == id) return &v;
        return nullptr;
    }

    // 一次性全面 dump 首个持续死锁簇:每个成员的路径要点 + 簇内两两冲突几何(same_dir 决定
    // 对向/同向 → 判定单向环流能否治)。只打一次,只读。用于源头修复的精确诊断。
    void dumpDeadlockCluster(const std::set<int>& members) {
        ROS_ERROR("[CLUSTER-DUMP] ===== 死锁簇 %zu 车,逐车路径 + 两两冲突几何 =====",
                  members.size());
        // 1) 每个成员:槽位、当前位姿/进度、路径起讫 + 等弧长 11 点折线。
        for (int id : members) {
            const VehicleAgent* a = agentById_c(id);
            if (!a || a->track.empty()) continue;
            const double L = a->track.length();
            const RoughWp cur = a->track.poseAtS(a->path_s);
            const RoughWp p0 = a->track.poseAtS(0.0);
            const RoughWp pe = a->track.poseAtS(L);
            ROS_ERROR("[CLUSTER-DUMP] V%d slot=%d->%d mode=%d act=%d reason=%s blk=%d "
                      "s=%.3f/%.3f cur(%.2f,%.2f,%.0fdeg) start(%.2f,%.2f) end(%.2f,%.2f)",
                      id, a->current_slot, a->target_slot, (int)a->mode, (int)a->action,
                      a->reason.c_str(), a->blocker_id, a->path_s, L,
                      cur.x, cur.y, cur.theta * 180.0 / M_PI, p0.x, p0.y, pe.x, pe.y);
            std::string poly;
            for (int k = 0; k <= 10; ++k) {
                const RoughWp p = a->track.poseAtS(L * k / 10.0);
                char buf[48];
                snprintf(buf, sizeof(buf), "(%.2f,%.2f%s)", p.x, p.y,
                         a->track.typeAtS(L * k / 10.0) == WpType::REVERSE ? "R" : "");
                poly += buf;
                if (k < 10) poly += "->";
            }
            ROS_ERROR("[CLUSTER-DUMP]   V%d path: %s", id, poly.c_str());
        }
        // 2) 簇内每对成员的冲突几何(same_dir / 物理坐标 / committed)。
        std::vector<int> ids(members.begin(), members.end());
        for (size_t i = 0; i < ids.size(); ++i) {
            for (size_t j = i + 1; j < ids.size(); ++j) {
                const VehicleAgent* a = agentById_c(ids[i]);
                const VehicleAgent* b = agentById_c(ids[j]);
                if (a && b) rule_engine_->debugDumpConflict(*a, *b);
            }
        }
        ROS_ERROR("[CLUSTER-DUMP] ===== 簇 dump 结束 =====");
    }

    // 持久 onset 文件:把关键现场同时写到 forklift_planner/logs。
    // 长测排错专用——只在出问题那一刻写,故文件小、不刷屏。
    void onsetLog(const std::string& s) {
        const std::string console_line = contextualLog(
            s, "REAL", coord_log_plan_id_, coord_log_frame_id_, -1);
        ROS_ERROR("%s", console_line.c_str());
        coordLog(s);
        std::ofstream f(onset_log_file_, std::ios::app);
        if (f) f << s << "\n";
    }
    // 把碰撞/楔死前的全队历史(含 gen=path_gen:刚被 recovery 重规划过则 gen 跳变=churn 撞)
    // 写进持久文件,供事后根因。
    void onsetDumpHist(const std::string& header, const std::deque<std::string>& hist) {
        std::ofstream f(onset_log_file_, std::ios::app);
        if (!f) return;
        f << "\n========== " << header << " ==========\n";
        coordLog("========== " + header + " ==========");
        for (const std::string& snap : hist) f << snap << "\n";
        for (const std::string& snap : hist) coordLog(snap);
        f.flush();
    }

    // 找「所有」持续死锁环的成员并集:对每辆车跟 blocker 链,若绕回自身则其环成员全部入集。
    std::set<int> findDeadlockMembers(double min_wait) const {
        auto idxOf = [&](int id) -> int {
            for (size_t i = 0; i < agents_.size(); ++i)
                if (agents_[i].id == id) return static_cast<int>(i);
            return -1;
        };
        std::set<int> members;
        for (const VehicleAgent& v0 : agents_) {
            if (v0.mode != VehicleMode::ACTIVE) continue;
            std::vector<int> path;
            int cur = v0.id;
            while (cur >= 0) {
                auto it = std::find(path.begin(), path.end(), cur);
                if (it != path.end()) {  // 绕回 → 环 = [it, end)
                    bool all_stuck = (path.end() - it) >= 2;
                    for (auto p = it; p != path.end(); ++p) {
                        int k = idxOf(*p);
                        if (k < 0 || agents_[k].wait_time < min_wait) { all_stuck = false; break; }
                    }
                    if (all_stuck) for (auto p = it; p != path.end(); ++p) members.insert(*p);
                    break;
                }
                path.push_back(cur);
                int k = idxOf(cur);
                if (k < 0 || agents_[k].mode != VehicleMode::ACTIVE ||
                    agents_[k].action != VehicleAction::STOP)
                    break;
                cur = agents_[k].blocker_id;
            }
        }
        return members;
    }

    // 批处理模式:紧凑循环跑 ticks 拍(跳过 marker)。维护近 N 拍环形历史;首次碰撞那拍
    // dump 全队历史+碰撞对几何,并对其后 kPost 拍逐拍详打;结尾 dump 永久楔死现场。
    bool runBatch(unsigned long long ticks) {
        const double dt = 1.0 / pp_.update_rate;
        std::ofstream(onset_log_file_, std::ios::trunc);
        const unsigned long long progress = ticks / 10 ? ticks / 10 : 1;
        constexpr size_t kHist = 80;    // 碰撞前回看的拍数
        constexpr unsigned long long kPost = 150;  // 碰撞后逐拍详打的拍数
        std::deque<std::string> hist;
        bool first_dumped = false;
        bool wedge_dumped = false;
        bool multiwedge_dumped = false;
        unsigned long long verbose_until = 0;

        for (unsigned long long k = 0; k < ticks && ros::ok(); ++k) {
            ++tick_count_;
            sim_time_ += dt;
            updateDwellAndTasks(dt);
            if (simulationPlanNeedsRefresh()) {
                std::vector<sandbox_msgs::Trajectory> trajs;
                std::vector<bool> hold;
                buildSimulationHorizonPlan(trajs, hold);
                force_horizon_refresh_ = false;
            }
            if (!executeSimulationPlanSample()) {
                std::vector<sandbox_msgs::Trajectory> trajs;
                std::vector<bool> hold;
                buildSimulationHorizonPlan(trajs, hold);
                force_horizon_refresh_ = false;
                if (!executeSimulationPlanSample()) {
                    rule_engine_->decide(agents_, dt);
                }
            }
            applyClusterAdmissionCounterfactual(dt);
            const unsigned long long guards_before = hard_guard_events_;
            advanceVehicles(dt);
            diagnoseA1ExitIntrusions();
            const bool new_collision = hard_guard_events_ > guards_before;

            for (const VehicleAgent& vehicle : agents_) {
                if (vehicle.action == VehicleAction::STOP) {
                    ++batch_stop_vehicle_ticks_;
                }
                batch_wait_sum_ += vehicle.wait_time;
                ++batch_wait_samples_;
                if (vehicle.wait_time > batch_max_observed_wait_) {
                    batch_max_observed_wait_ = vehicle.wait_time;
                    batch_max_observed_wait_vehicle_ = vehicle.id;
                }
            }
            const bool deadlock_now =
                !findDeadlockMembers(25.0).empty();
            if (deadlock_now && !batch_deadlock_episode_active_) {
                ++batch_deadlock_episodes_;
            }
            batch_deadlock_episode_active_ = deadlock_now;

            hist.push_back(fleetSnapshot());
            if (hist.size() > kHist) hist.pop_front();

            if (new_collision && !first_dumped) {
                first_dumped = true;
                verbose_until = tick_count_ + kPost;
                std::string cp;
                for (const auto& q : hard_guard_pairs_)
                    cp += "V" + std::to_string(q.first) + "-V" +
                          std::to_string(q.second) + " ";
                onsetLog("[FIRST-COLLISION] @tick=" + std::to_string(tick_count_) +
                         " sim_t=" + std::to_string((long long)sim_time_) +
                         "s 涉及对=[" + cp + "]");
                // churn 信号:每个涉事车「距上次脱困重规划多久」——刚被重规划(Δt 很小)= 很可能
                // 是 recovery 把它重置成与他车重叠的新 track 撞出来的。
                for (const auto& q : hard_guard_pairs_) {
                    for (int id : {q.first, q.second}) {
                        auto it = last_replan_t_.find(id);
                        const double dt_re = (it == last_replan_t_.end())
                            ? -1.0 : (sim_time_ - it->second);
                        const VehicleAgent* a = agentById_c(id);
                        onsetLog("[FIRST-COLLISION]   V" + std::to_string(id) +
                                 " 距上次脱困=" + (dt_re < 0 ? std::string("从未") :
                                     std::to_string((long long)dt_re) + "s") +
                                 " gen=" + std::to_string(a ? a->path_gen : -1) +
                                 " slot=" + std::to_string(a ? a->current_slot : -1) +
                                 "->" + std::to_string(a ? a->target_slot : -1));
                    }
                }
                onsetDumpHist("FIRST-COLLISION 前 " + std::to_string(hist.size()) +
                              " 拍历史(看 gen 是否刚跳变=刚被脱困重置)", hist);
                ROS_WARN("[HIST] ====== 碰撞前 %zu 拍全队历史 ======", hist.size());
                for (const std::string& snap : hist) ROS_WARN("[HIST]\n%s", snap.c_str());
                ROS_WARN("[HIST] ====== 碰撞对冲突几何 ======");
                for (const auto& q : hard_guard_pairs_) dumpPair(q.first, q.second);
            }

            // 多车紧楔 onset(24h 杀手):≥3 车持续闭环(wait≥15s)首次出现 → 一次性 dump 到
            // 持久文件 + 历史。这是 B(40s前瞻)够不着、recovery 治不了的那类,要抓它怎么形成的。
            if (!multiwedge_dumped) {
                const std::set<int> mw3 = findDeadlockMembers(15.0);
                if (mw3.size() >= 3) {
                    multiwedge_dumped = true;
                    std::string ms;
                    for (int id : mw3) ms += "V" + std::to_string(id) + " ";
                    onsetLog("[MULTIWEDGE] @tick=" + std::to_string(tick_count_) +
                             " sim_t=" + std::to_string((long long)sim_time_) +
                             "s ≥3车紧楔首现 成员=[" + ms + "]");
                    onsetDumpHist("MULTIWEDGE 前 " + std::to_string(hist.size()) + " 拍历史", hist);
                    dumpDeadlockCluster(mw3);
                }
            }

            // 楔死现场一次性诊断:任一车 wait 首次超阈值 → 回放历史 + 全队 + 卡死车几何。
            if (!wedge_dumped) {
                int sid = -1; double mw = 0.0;
                for (const VehicleAgent& v : agents_)
                    if (v.wait_time > mw) { mw = v.wait_time; sid = v.id; }
                if (mw > 25.0 && sid >= 0) {
                    wedge_dumped = true;
                    ++batch_first_wedge_count_;
                    ROS_ERROR("[FIRST-WEDGE] @tick=%llu sim_t=%.1fs 最久=V%d wait=%.1fs"
                              " —— 回放前 %zu 拍历史 + 卡死车几何 ===",
                              tick_count_, sim_time_, sid, mw, hist.size());
                    for (const std::string& snap : hist)
                        ROS_WARN("[WHIST]\n%s", snap.c_str());
                    for (const VehicleAgent& v : agents_)
                        if (v.id != sid) dumpPair(sid, v.id);
                }
            }

            if (tick_count_ <= verbose_until) {
                ROS_WARN("[POST]\n%s", fleetSnapshot().c_str());
                for (const auto& q : hard_guard_pairs_) dumpPair(q.first, q.second);
            }

            // 死锁看门狗(C):检测持续环 → 受害车从当前位姿重规划脱困。每 5 拍查一次。
            if (k % 5 == 0) runDeadlockRecovery();

            if ((k + 1) % progress == 0) {
                ROS_INFO("[batch] %llu/%llu ticks (sim_t=%.0fs) hard_guard=%llu",
                         k + 1, ticks, sim_time_, hard_guard_events_);
            }
        }

        // 结尾:找等待最久(永久楔死)的车,dump 它与所有其它车的冲突几何 + 全队快照。
        double max_wait = 0.0;
        int stuck_id = -1;
        for (const VehicleAgent& v : agents_)
            if (v.wait_time > max_wait) { max_wait = v.wait_time; stuck_id = v.id; }
        ROS_WARN("[END] ====== 结尾全队快照 ======\n%s", fleetSnapshot().c_str());
        if (stuck_id >= 0 && max_wait > 20.0) {
            ROS_WARN("[END] 永久楔死嫌疑=V%d (wait=%.1fs),其与各车冲突几何:",
                     stuck_id, max_wait);
            for (const VehicleAgent& v : agents_)
                if (v.id != stuck_id) dumpPair(stuck_id, v.id);
        }
        std::string pairs;
        for (const auto& p : hard_guard_pairs_)
            pairs += "V" + std::to_string(p.first) + "-V" +
                     std::to_string(p.second) + " ";
        ROS_WARN("[batch] ==== 汇总: ticks=%llu sim_t=%.0fs | 碰撞(hard_guard)事件=%llu "
                 "首次@tick=%llu 涉及对=[%s] | 死锁检出拍=%llu 重规划脱困=%llu | "
                 "最大wait=%.1fs(V%d) ====",
                 tick_count_, sim_time_, hard_guard_events_, first_guard_tick_,
                 pairs.c_str(), deadlock_ticks_, deadlock_recoveries_, max_wait,
                 stuck_id);
        std::ostringstream tasks;
        tasks << "[";
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            if (i > 0) tasks << ",";
            tasks << "V" << agents_[i].id << ":"
                  << agents_[i].task_count;
        }
        tasks << "]";
        const double average_wait = batch_wait_samples_ > 0
            ? batch_wait_sum_ / static_cast<double>(batch_wait_samples_)
            : 0.0;
        const char* regression_source =
            cluster_counterfactual_shadow_enabled_
                ? "COUNTERFACTUAL" : "REAL";
        ROS_WARN("[PHASE62_REGRESSION] source=%s tasks=%s "
                 "max_wait=%.3f average_vehicle_tick_wait=%.6f "
                 "stop_vehicle_ticks=%llu first_wedge_events=%llu "
                 "deadlock_episodes=%llu deadlock_detection_ticks=%llu "
                 "hard_guard_collisions=%llu recovery=%llu",
                 regression_source, tasks.str().c_str(),
                 batch_max_observed_wait_, average_wait,
                 batch_stop_vehicle_ticks_, batch_first_wedge_count_,
                 batch_deadlock_episodes_, deadlock_ticks_,
                 hard_guard_events_, deadlock_recoveries_);
        ROS_WARN("[PHASE62_REGRESSION] source=SHADOW "
                 "admission_evaluations=%llu admission_valid=%llu "
                 "prevent_zone_mixing=%llu late_mixing_observed=%llu "
                 "closed_loop_tasks=unknown closed_loop_wait=unknown "
                 "closed_loop_deadlock=unknown",
                 cluster_admission_evaluations_,
                 cluster_admission_valid_count_,
                 cluster_admission_prevent_count_,
                 cluster_admission_late_mixing_count_);
        ROS_WARN("[PHASE63_REGRESSION] source=%s sim_time=%.1f "
                 "tasks=%s max_continuous_wait=%.3f(V%d) "
                 "average_vehicle_tick_wait=%.6f stop_vehicle_ticks=%llu "
                 "first_wedge_events=%llu deadlock_episodes=%llu "
                 "deadlock_detection_ticks=%llu hard_guard_collisions=%llu "
                 "recovery=%llu shadow_stop_ticks=%llu go_overrides=%llu "
                 "cluster_releases=%llu waiter_resumes=%llu "
                 "waiter_entry_violations=%llu late_braking_events=%llu "
                 "dynamic_feasible=%llu dynamic_infeasible=%llu "
                 "dynamic_stop_requests=%llu dynamic_action_changes=%llu "
                 "max_cluster_wait=%.3f",
                 regression_source, sim_time_, tasks.str().c_str(),
                 batch_max_observed_wait_, batch_max_observed_wait_vehicle_,
                 average_wait, batch_stop_vehicle_ticks_,
                 batch_first_wedge_count_, batch_deadlock_episodes_,
                 deadlock_ticks_, hard_guard_events_, deadlock_recoveries_,
                 counterfactual_stop_vehicle_ticks_,
                 counterfactual_go_overrides_,
                 counterfactual_cluster_releases_,
                 counterfactual_waiter_resumes_,
                 counterfactual_waiter_entry_violations_,
                 counterfactual_late_braking_events_,
                 dynamic_admission_feasible_count_,
                 dynamic_admission_infeasible_count_,
                 dynamic_admission_stop_requests_,
                 dynamic_admission_action_changes_,
                 counterfactual_max_cluster_wait_);
        return hard_guard_events_ > 0;
    }
    bool batchMode() const { return cfg_batch_ticks_ > 0; }
    unsigned long long batchTicks() const { return cfg_batch_ticks_; }

private:
    unsigned long long cfg_batch_ticks_ = 0;
};

int main(int argc, char** argv) {
    // 用环境 locale(通常 UTF-8)初始化 C/C++ 本地化,否则默认 "C" locale 会把日志里的
    // 中文打成 ???。一句即可,根治 rosconsole/printf 中文乱码。
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "multi_vehicle_patrol_node");
    MultiVehiclePatrolNode node;

    if (node.batchMode()) {
        // 无头快速回归:狂跑后退出。返回码 1=出现碰撞,0=干净(便于脚本判定)。
        const bool collided = node.runBatch(node.batchTicks());
        ros::shutdown();
        return collided ? 1 : 0;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
