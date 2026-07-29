#include "forklift_planner/multi_vehicle/marker_publisher.h"

#include <geometry_msgs/Point.h>

#include <algorithm>
#include <cmath>
#include <sstream>

#include "forklift_planner/multi_vehicle/footprint.h"

namespace forklift_planner {
namespace multi_vehicle {

namespace {

geometry_msgs::Point pt3(double x, double y, double z = 0.08) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

RoughWp displayPose(const VehicleAgent& v) {
    // 实车:显示真实 /object 位姿(实际在哪就画在哪,偏离路径多少看得见),而非投影点。
    if (v.real_pose_valid) {
        RoughWp p; p.x = v.real_x; p.y = v.real_y; p.theta = v.real_yaw;
        p.type = WpType::FORWARD;
        return p;
    }
    if (v.track.empty()) return {};
    if (v.mode == VehicleMode::DWELL) return v.track.poseAtS(v.track.length());
    return v.track.poseAtS(v.path_s);
}

void shelfCellY(const MapParam& p, int row_id, double& y0, double& y1) {
    constexpr double kSlotGapHalf = 0.012;
    const double g = kSlotGapHalf;
    switch (row_id) {
        case 0: y0 = p.y8();                  y1 = p.field_height;          break;
        case 1: y0 = (p.y6()+p.y7())*0.5 + g; y1 = p.y7();                  break;
        case 2: y0 = p.y6();                  y1 = (p.y6()+p.y7())*0.5 - g; break;
        case 3: y0 = (p.y4()+p.y5())*0.5 + g; y1 = p.y5();                  break;
        case 4: y0 = p.y4();                  y1 = (p.y4()+p.y5())*0.5 - g; break;
        case 5: y0 = (p.y2()+p.y3())*0.5 + g; y1 = p.y3();                  break;
        case 6: y0 = p.y2();                  y1 = (p.y2()+p.y3())*0.5 - g; break;
        default:y0 = 0.0;                     y1 = p.bottom_shelf_depth;    break;
    }
}

}  // namespace

MarkerPublisher::MarkerPublisher(ros::NodeHandle& nh, const MapParam& mp,
                                 const PlannerParam& pp,
                                 const std::vector<Slot>& slots,
                                 const MultiVehicleConfig& cfg)
    : mp_(mp), pp_(pp), slots_(slots), cfg_(cfg) {
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/forklift_planner/markers", 10);
}

void MarkerPublisher::addPathMarker(visualization_msgs::MarkerArray& arr,
                                    const VehicleAgent& v) const {
    constexpr int kPathPublishStride = 10;
    if (cfg_.show_paths && publish_seq_ % kPathPublishStride != 0) {
        return;
    }

    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_path";
    m.id = v.id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = cfg_.show_paths ? visualization_msgs::Marker::ADD
                               : visualization_msgs::Marker::DELETE;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.016;
    m.color = v.color;
    if (cfg_.show_paths) {
        for (const RoughWp& p : v.track.path()) {
            m.points.push_back(pt3(p.x, p.y, 0.045));
        }
    }
    arr.markers.push_back(m);
}

void MarkerPublisher::addBodyMarker(visualization_msgs::MarkerArray& arr,
                                    const VehicleAgent& v) const {
    // 路径点是后轴参考；车身方块画在车身几何中心。
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    const double c = std::cos(p.theta);
    const double s = std::sin(p.theta);
    const double L = mp_.vehicle_length;
    const double W = mp_.vehicle_width;

    // RViz 只改变外观，不改变规划/碰撞 footprint：
    // 外包络仍是 L×W；后半段是车身，前半段分成两根叉臂，便于区分车头。
    const double body_len = L * 0.62;
    const double fork_len = L - body_len;
    const double fork_w = W * 0.22;
    const double body_x = -0.5 * L + 0.5 * body_len;
    const double fork_x = 0.5 * L - 0.5 * fork_len;
    const double fork_y = 0.5 * W - 0.5 * fork_w;

    auto addPart = [&](int id, double local_x, double local_y,
                       double sx, double sy) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "multi_patrol_body";
        m.id = id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = p.x + c * local_x - s * local_y;
        m.pose.position.y = p.y + s * local_x + c * local_y;
        m.pose.position.z = 0.035;
        m.pose.orientation.z = std::sin(p.theta * 0.5);
        m.pose.orientation.w = std::cos(p.theta * 0.5);
        m.scale.x = sx;
        m.scale.y = sy;
        m.scale.z = 0.050;
        m.color = v.color;
        arr.markers.push_back(m);
    };

    addPart(v.id, body_x, 0.0, body_len, W);
    addPart(1000 + v.id * 2, fork_x, fork_y, fork_len, fork_w);
    addPart(1001 + v.id * 2, fork_x, -fork_y, fork_len, fork_w);

    // A loaded forklift carries a visible pallet/cargo block over its forks.
    // Publish DELETE when empty so the previous marker disappears immediately
    // after unloading.
    visualization_msgs::Marker cargo;
    cargo.header.frame_id = pp_.frame_id;
    cargo.header.stamp = ros::Time::now();
    cargo.ns = "multi_patrol_cargo";
    cargo.id = 2000 + v.id;
    cargo.type = visualization_msgs::Marker::CUBE;
    cargo.action = v.loaded ? visualization_msgs::Marker::ADD
                            : visualization_msgs::Marker::DELETE;
    if (v.loaded) {
        cargo.pose.position.x = p.x + c * fork_x;
        cargo.pose.position.y = p.y + s * fork_x;
        cargo.pose.position.z = 0.085;
        cargo.pose.orientation.z = std::sin(p.theta * 0.5);
        cargo.pose.orientation.w = std::cos(p.theta * 0.5);
        cargo.scale.x = fork_len * 0.82;
        cargo.scale.y = W * 0.72;
        cargo.scale.z = 0.085;
        cargo.color = rgba(0.72f, 0.43f, 0.14f, 0.98f);
    }
    arr.markers.push_back(cargo);
}

void MarkerPublisher::addArrowMarker(visualization_msgs::MarkerArray& arr,
                                     const VehicleAgent& v) const {
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_arrow";
    m.id = v.id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.010;
    m.scale.y = 0.022;
    m.scale.z = 0.0;
    m.color = v.color;

    const double half = mp_.vehicle_length * 0.55;
    const double dx = std::cos(p.theta) * half;
    const double dy = std::sin(p.theta) * half;
    m.points.push_back(pt3(p.x - dx * 0.5, p.y - dy * 0.5, 0.070));
    m.points.push_back(pt3(p.x + dx, p.y + dy, 0.070));
    arr.markers.push_back(m);
}

void MarkerPublisher::addLabelMarker(visualization_msgs::MarkerArray& arr,
                                     const VehicleAgent& v) const {
    const RoughWp p = bodyCenterPose(displayPose(v), mp_);
    visualization_msgs::Marker m;
    m.header.frame_id = pp_.frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "multi_patrol_label";
    m.id = v.id;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = 0.160;
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.070;
    m.color = v.color;
    m.text = "V" + std::to_string(v.id) + " " + actionName(v.action);
    if (v.mode == VehicleMode::WAIT_DISPATCH) {
        m.text += " WAIT_B";
    } else if (v.mode == VehicleMode::DWELL) {
        m.text += " DWELL";
    } else if (v.loaded) {
        m.text += " L";
    } else {
        m.text += " E";
    }
    if (v.mode == VehicleMode::ACTIVE) {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(2);
        ss << " v=" << v.current_speed;
        if (!v.reason.empty()) {
            ss << " " << v.reason;
        }
        m.text += ss.str();
    }
    arr.markers.push_back(m);
}

void MarkerPublisher::addVisitedSlotMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<bool>& visited_slots) const {
    const size_t n = std::min(visited_slots.size(), slots_.size());
    for (size_t i = 0; i < n; ++i) {
        if (!visited_slots[i]) continue;
        const Slot& s = slots_[i];
        double y0 = 0.0;
        double y1 = 0.0;
        shelfCellY(mp_, s.row_id, y0, y1);

        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "visited_slots";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = s.cx;
        m.pose.position.y = 0.5 * (y0 + y1);
        m.pose.position.z = 0.018;
        m.pose.orientation.w = 1.0;
        m.scale.x = mp_.vehicle_width;
        m.scale.y = y1 - y0;
        m.scale.z = 0.004;
        m.color = rgba(1.0f, 0.88f, 0.05f, 0.92f);
        arr.markers.push_back(m);
    }
}

void MarkerPublisher::addA1ServiceZoneMarkers(
    visualization_msgs::MarkerArray& arr) const {
    if (!cfg_.use_a1_cycle) return;

    // The hard exclusion marker uses the exact same rectangles as the rule
    // engine. It is deliberately red-orange and distinct from conflict zones.
    const double queue_y = 0.5 * (mp_.y6() + mp_.y7());
    const double right_x0 = mp_.row1_left_aisle + mp_.row1_shelf_width;
    const double right_x1 = mp_.field_width - mp_.row1_mini_shelf;
    const std_msgs::ColorRGBA fill =
        rgba(1.00f, 0.20f, 0.10f, 0.15f);
    const std_msgs::ColorRGBA edge =
        rgba(1.00f, 0.30f, 0.10f, 0.95f);

    auto addBox = [&](int id, double x0, double x1,
                      double y0, double y1) {
        if (x1 <= x0 || y1 <= y0) return;
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "a1_service_zone";
        m.id = id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 0.5 * (x0 + x1);
        m.pose.position.y = 0.5 * (y0 + y1);
        m.pose.position.z = 0.007;
        m.pose.orientation.w = 1.0;
        m.scale.x = x1 - x0;
        m.scale.y = y1 - y0;
        m.scale.z = 0.006;
        m.color = fill;
        arr.markers.push_back(m);
    };
    int zone_id = 0;
    for (const A1HardZoneRect& r : a1HardZoneRects(mp_)) {
        addBox(zone_id++, r.x0, r.x1, r.y0, r.y1);
    }

    auto addQueueLine = [&](int id, double x0, double x1) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "a1_service_queue_line";
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.026;
        m.color = edge;
        m.points.push_back(pt3(x0, queue_y, 0.026));
        m.points.push_back(pt3(x1, queue_y, 0.026));
        arr.markers.push_back(m);
    };
    addQueueLine(0, 0.0, mp_.row1_left_aisle);
    addQueueLine(1, right_x0, right_x1);

    visualization_msgs::Marker label;
    label.header.frame_id = pp_.frame_id;
    label.header.stamp = ros::Time::now();
    label.ns = "a1_service_zone";
    label.id = 10;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = 0.5 * mp_.field_width;
    label.pose.position.y =
        mp_.field_height - 0.5 * mp_.bottom_shelf_depth;
    label.pose.position.z = 0.115;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.085;
    label.color = edge;
    label.text = "A1 HARD EXCLUSION";
    arr.markers.push_back(label);
}

void MarkerPublisher::addConflictMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<ConflictMarker>& conflicts) const {
    const char* same_ns = "conflict_same_direction";
    const char* mutual_ns = "conflict_crossing_or_opposing";
    const char* a1_ns = "conflict_a1_protected";
    auto deleteMarkers = [&](const char* marker_ns, int count) {
        for (int id = 0; id < count; ++id) {
            visualization_msgs::Marker m;
            m.header.frame_id = pp_.frame_id;
            m.header.stamp = ros::Time::now();
            m.ns = marker_ns;
            m.id = id;
            m.action = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(m);
        }
    };

    if (!cfg_.show_prediction_conflicts) {
        deleteMarkers(same_ns,
                      last_same_direction_conflict_marker_count_);
        deleteMarkers(mutual_ns,
                      last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(a1_ns,
                      last_a1_protected_conflict_marker_count_);
        last_same_direction_conflict_marker_count_ = 0;
        last_crossing_opposing_conflict_marker_count_ = 0;
        last_a1_protected_conflict_marker_count_ = 0;
        return;
    }

    int same_id = 0;
    int mutual_id = 0;
    int a1_id = 0;
    for (const ConflictMarker& c : conflicts) {
        const bool same_direction =
            c.kind == ConflictMarkerKind::SAME_DIRECTION;
        const bool a1_protected =
            c.kind == ConflictMarkerKind::A1_PROTECTED;
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        if (a1_protected) {
            m.ns = a1_ns;
            m.id = a1_id++;
        } else if (same_direction) {
            m.ns = same_ns;
            m.id = same_id++;
        } else {
            m.ns = mutual_ns;
            m.id = mutual_id++;
        }
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = 0.018;
        m.pose.orientation.w = 1.0;
        m.scale.x = c.scale_x;
        m.scale.y = c.scale_y;
        m.scale.z = 0.012;
        if (a1_protected) {
            // Light purple: exact blocks belonging to the authoritative A1
            // approach/load/departure transaction.
            m.color = rgba(0.78f, 0.48f, 1.00f, 0.34f);
        } else if (same_direction) {
            // Light cyan: longitudinal following arbitration.
            m.color = rgba(0.35f, 0.82f, 1.00f, 0.28f);
        } else {
            // Light orange: crossing and opposing traffic share the same
            // mutual-exclusion holder arbitration.
            m.color = rgba(1.00f, 0.62f, 0.34f, 0.30f);
        }
        arr.markers.push_back(m);
    }

    for (int stale = same_id;
         stale < last_same_direction_conflict_marker_count_; ++stale) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = same_ns;
        m.id = stale;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    }
    for (int stale = mutual_id;
         stale < last_crossing_opposing_conflict_marker_count_; ++stale) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = mutual_ns;
        m.id = stale;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    }
    for (int stale = a1_id;
         stale < last_a1_protected_conflict_marker_count_; ++stale) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = a1_ns;
        m.id = stale;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    }
    last_same_direction_conflict_marker_count_ = same_id;
    last_crossing_opposing_conflict_marker_count_ = mutual_id;
    last_a1_protected_conflict_marker_count_ = a1_id;
}

void MarkerPublisher::addA1GateMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<A1GateMarker>& a1_gates) const {
    const char* gate_ns = "a1_authoritative_gate";
    int marker_id = 0;
    auto sourceName = [](A1GateSource source) {
        switch (source) {
            case A1GateSource::FIXED: return "FIXED";
            case A1GateSource::VERTICAL_QUEUE: return "VERT";
            case A1GateSource::APPROACH_CHAIN: return "APP";
            case A1GateSource::PENDING_EXIT_CHAIN: return "P-EXIT";
            case A1GateSource::ACTIVE_EXIT_CHAIN: return "A-EXIT";
        }
        return "?";
    };

    for (const A1GateMarker& gate : a1_gates) {
        const std_msgs::ColorRGBA color =
            gate.late ? rgba(1.00f, 0.16f, 0.26f, 0.98f)
                      : rgba(0.90f, 0.58f, 1.00f, 0.98f);
        const double half_width =
            std::max(0.12, 0.75 * mp_.vehicle_width);
        const double dx = -std::sin(gate.theta) * half_width;
        const double dy = std::cos(gate.theta) * half_width;

        visualization_msgs::Marker line;
        line.header.frame_id = pp_.frame_id;
        line.header.stamp = ros::Time::now();
        line.ns = gate_ns;
        line.id = marker_id++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.034;
        line.color = color;
        line.points.push_back(pt3(gate.x - dx, gate.y - dy, 0.034));
        line.points.push_back(pt3(gate.x + dx, gate.y + dy, 0.034));
        arr.markers.push_back(line);

        visualization_msgs::Marker label;
        label.header.frame_id = pp_.frame_id;
        label.header.stamp = ros::Time::now();
        label.ns = gate_ns;
        label.id = marker_id++;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::Marker::ADD;
        label.pose.position.x = gate.x;
        label.pose.position.y = gate.y;
        label.pose.position.z = 0.105;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.052;
        label.color = color;
        std::ostringstream text;
        text << "A1 GATE V" << gate.waiter_id << "->V" << gate.owner_id
             << " " << sourceName(gate.source)
             << " A" << gate.approach_zone_count
             << "/D" << gate.departure_zone_count;
        label.text = text.str();
        arr.markers.push_back(label);
    }

    for (int stale = marker_id; stale < last_a1_gate_marker_count_; ++stale) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = gate_ns;
        m.id = stale;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    }
    last_a1_gate_marker_count_ = marker_id;
}

void MarkerPublisher::addOriginAxes(visualization_msgs::MarkerArray& arr) const {
    // 地图原点 (0,0) 与 X/Y 正方向。实车标定时:动捕 world 原点应与此重合、
    // 某车放已知槽位时 /object÷1000 应≈该槽坐标,车头朝 +X(红轴)时 yaw≈0。
    constexpr double L = 0.5;   // 轴长 0.5m(地图 ~2.5×4.5,够看又不挡)
    const double z = 0.075;     // 抬到与车身箭头同高(z=0.07),否则被地图平面盖住看不见
    auto axis = [&](int id, double ex, double ey, const std_msgs::ColorRGBA& col) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "map_origin_axes";
        m.id = id;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.022;   // 杆径(加粗,醒目)
        m.scale.y = 0.05;    // 箭头径
        m.scale.z = 0.08;    // 箭头长
        m.color = col;
        m.points.push_back(pt3(0.0, 0.0, z));
        m.points.push_back(pt3(ex * L, ey * L, z));
        arr.markers.push_back(m);
    };
    auto label = [&](int id, double x, double y, const std::string& txt,
                     const std_msgs::ColorRGBA& col) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "map_origin_axes";
        m.id = id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.14;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.10;
        m.color = col;
        m.text = txt;
        arr.markers.push_back(m);
    };
    axis(0, 1.0, 0.0, rgba(0.95f, 0.1f, 0.1f));   // +X 红
    axis(1, 0.0, 1.0, rgba(0.1f, 0.9f, 0.1f));    // +Y 绿
    label(2, L + 0.06, 0.0, "+X", rgba(0.95f, 0.1f, 0.1f));
    label(3, 0.0, L + 0.06, "+Y", rgba(0.1f, 0.9f, 0.1f));
    label(4, -0.06, -0.06, "O(0,0)", rgba(1.0f, 1.0f, 1.0f));
}

void MarkerPublisher::publish(
    const std::vector<VehicleAgent>& vehicles,
    const std::vector<bool>& visited_slots,
    const std::vector<ConflictMarker>& conflicts,
    const std::vector<A1GateMarker>& a1_gates) const {
    ++publish_seq_;
    visualization_msgs::MarkerArray arr;
    addA1ServiceZoneMarkers(arr);
    addVisitedSlotMarkers(arr, visited_slots);
    addOriginAxes(arr);  // 地图原点+XY正方向(标定核对用)
    for (const VehicleAgent& v : vehicles) {
        addPathMarker(arr, v);
        if (v.mode == VehicleMode::NEED_TASK || v.track.empty()) continue;
        addBodyMarker(arr, v);
        addArrowMarker(arr, v);
        addLabelMarker(arr, v);
    }
    addConflictMarkers(arr, conflicts);
    addA1GateMarkers(arr, a1_gates);
    pub_.publish(arr);
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
