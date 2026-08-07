#include "forklift_planner/multi_vehicle/marker_publisher.h"

#include <geometry_msgs/Point.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
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
                                 const MultiVehicleConfig& cfg,
                                 const Slot& a1_pickup)
    : mp_(mp), pp_(pp), slots_(slots), cfg_(cfg),
      a1_pickup_(a1_pickup) {
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/forklift_planner/markers", 10);
}

void MarkerPublisher::addA1PickupDefinitionMarkers(
    visualization_msgs::MarkerArray& arr) const {
    if (!cfg_.use_a1_cycle) return;
    const ros::Time now = ros::Time::now();

    // Complete parked vehicle body at the physical pickup pose. The path
    // endpoint itself remains a rear-axle reference and is intentionally not
    // used as the visual pickup footprint.
    visualization_msgs::Marker pickup;
    pickup.header.frame_id = pp_.frame_id;
    pickup.header.stamp = now;
    pickup.ns = "a1_pickup_definition";
    pickup.id = 0;
    pickup.type = visualization_msgs::Marker::CUBE;
    pickup.action = visualization_msgs::Marker::ADD;
    pickup.pose.position.x = a1_pickup_.cx;
    pickup.pose.position.y = a1_pickup_.cy;
    pickup.pose.position.z = 0.026;
    pickup.pose.orientation.z = std::sin(0.5 * a1_pickup_.dock_theta);
    pickup.pose.orientation.w = std::cos(0.5 * a1_pickup_.dock_theta);
    pickup.scale.x = mp_.vehicle_length;
    pickup.scale.y = mp_.vehicle_width;
    pickup.scale.z = 0.020;
    pickup.color = rgba(0.20f, 1.00f, 0.30f, 0.42f);
    arr.markers.push_back(pickup);

    visualization_msgs::Marker direction;
    direction.header = pickup.header;
    direction.ns = "a1_pickup_definition";
    direction.id = 1;
    direction.type = visualization_msgs::Marker::ARROW;
    direction.action = visualization_msgs::Marker::ADD;
    direction.pose.orientation.w = 1.0;
    direction.scale.x = 0.018;
    direction.scale.y = 0.045;
    direction.scale.z = 0.065;
    direction.color = rgba(0.20f, 1.00f, 0.30f, 1.0f);
    direction.points.push_back(pt3(a1_pickup_.cx, a1_pickup_.cy, 0.065));
    direction.points.push_back(pt3(
        a1_pickup_.cx + 0.24 * std::cos(a1_pickup_.dock_theta),
        a1_pickup_.cy + 0.24 * std::sin(a1_pickup_.dock_theta), 0.065));
    arr.markers.push_back(direction);

    visualization_msgs::Marker pre_dock;
    pre_dock.header = pickup.header;
    pre_dock.ns = "a1_pickup_definition";
    pre_dock.id = 2;
    pre_dock.type = visualization_msgs::Marker::SPHERE;
    pre_dock.action = visualization_msgs::Marker::ADD;
    pre_dock.pose.position.x = a1_pickup_.pre_dock_x;
    pre_dock.pose.position.y = a1_pickup_.pre_dock_y;
    pre_dock.pose.position.z = 0.045;
    pre_dock.pose.orientation.w = 1.0;
    pre_dock.scale.x = 0.055;
    pre_dock.scale.y = 0.055;
    pre_dock.scale.z = 0.055;
    pre_dock.color = rgba(1.0f, 1.0f, 1.0f, 0.95f);
    arr.markers.push_back(pre_dock);

    visualization_msgs::Marker label;
    label.header = pickup.header;
    label.ns = "a1_pickup_definition";
    label.id = 3;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = a1_pickup_.cx;
    label.pose.position.y = a1_pickup_.cy;
    label.pose.position.z = 0.19;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.075;
    label.color = rgba(0.20f, 1.00f, 0.30f, 1.0f);
    label.text = "A1 PICKUP (5s)";
    arr.markers.push_back(label);
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
    if (v.mode == VehicleMode::DWELL) {
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

void MarkerPublisher::addConflictMarkers(
    visualization_msgs::MarkerArray& arr,
    const std::vector<ConflictMarker>& conflicts) const {
    const char* same_ns = "conflict_same_direction";
    const char* mutual_ns = "conflict_crossing_or_opposing";
    const char* actual_ns = "conflict_actual_overlap_center";
    const char* conflict_label_ns = "conflict_explanation";
    const char* following_relation_ns = "following_relation";
    const char* following_label_ns = "following_explanation";
    auto deleteMarker = [&](const char* marker_ns, int id) {
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = marker_ns;
        m.id = id;
        m.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(m);
    };
    auto deleteMarkers = [&](const char* marker_ns, int count) {
        for (int id = 0; id < count; ++id) {
            deleteMarker(marker_ns, id);
        }
    };

    if (!cfg_.show_prediction_conflicts) {
        deleteMarkers(same_ns,
                      last_same_direction_conflict_marker_count_);
        deleteMarkers(following_relation_ns,
                      last_same_direction_conflict_marker_count_);
        deleteMarkers(following_label_ns,
                      last_same_direction_conflict_marker_count_);
        deleteMarkers(mutual_ns,
                      last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(actual_ns,
                      last_crossing_opposing_conflict_marker_count_);
        deleteMarkers(conflict_label_ns,
                      last_crossing_opposing_conflict_marker_count_);
        last_same_direction_conflict_marker_count_ = 0;
        last_crossing_opposing_conflict_marker_count_ = 0;
        return;
    }

    int same_id = 0;
    int mutual_id = 0;
    for (const ConflictMarker& c : conflicts) {
        const bool same_direction =
            c.kind == ConflictMarkerKind::SAME_DIRECTION;
        const ros::Time now = ros::Time::now();
        const int marker_id = same_direction ? same_id++ : mutual_id++;
        visualization_msgs::Marker m;
        m.header.frame_id = pp_.frame_id;
        m.header.stamp = now;
        m.ns = same_direction ? same_ns : mutual_ns;
        m.id = marker_id;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = 0.018;
        m.pose.orientation.w = 1.0;
        if (same_direction) {
            // An outline, not a filled collision patch: this is only the
            // spatial range used to explain the following relationship.
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.pose.position.x = 0.0;
            m.pose.position.y = 0.0;
            m.scale.x = 0.012;
            m.color = rgba(0.20f, 0.82f, 1.00f, 0.85f);
            const double x0 = c.x - 0.5 * c.scale_x;
            const double x1 = c.x + 0.5 * c.scale_x;
            const double y0 = c.y - 0.5 * c.scale_y;
            const double y1 = c.y + 0.5 * c.scale_y;
            m.points.push_back(pt3(x0, y0, 0.035));
            m.points.push_back(pt3(x1, y0, 0.035));
            m.points.push_back(pt3(x1, y1, 0.035));
            m.points.push_back(pt3(x0, y1, 0.035));
            m.points.push_back(pt3(x0, y0, 0.035));
        } else {
            // Enlarged AABB retained as a search/explanation range. It is not
            // the exact OBB overlap footprint.
            m.type = visualization_msgs::Marker::CUBE;
            m.scale.x = c.scale_x;
            m.scale.y = c.scale_y;
            m.scale.z = 0.012;
            m.color = rgba(1.00f, 0.62f, 0.34f, 0.20f);
        }
        arr.markers.push_back(m);

        if (same_direction) {
            visualization_msgs::Marker relation;
            relation.header = m.header;
            relation.ns = following_relation_ns;
            relation.id = marker_id;
            relation.type = visualization_msgs::Marker::ARROW;
            relation.action = visualization_msgs::Marker::ADD;
            relation.pose.orientation.w = 1.0;
            relation.scale.x = 0.015;
            relation.scale.y = 0.040;
            relation.scale.z = 0.055;
            relation.color = rgba(0.10f, 0.90f, 1.00f, 1.0f);
            relation.points.push_back(
                pt3(c.follower_x, c.follower_y, 0.075));
            relation.points.push_back(
                pt3(c.leader_x, c.leader_y, 0.075));
            arr.markers.push_back(relation);

            visualization_msgs::Marker label;
            label.header = m.header;
            label.ns = following_label_ns;
            label.id = marker_id;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position.x = 0.5 * (c.follower_x + c.leader_x);
            label.pose.position.y = 0.5 * (c.follower_y + c.leader_y);
            label.pose.position.z = 0.16;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.070;
            label.color = rgba(0.10f, 0.90f, 1.00f, 1.0f);
            std::ostringstream text;
            text << std::fixed << std::setprecision(2)
                 << "FOLLOW V" << c.follower_id << " -> V" << c.leader_id
                 << " gap=" << c.following_gap << "m"
                 << "\nrange outline (not collision)";
            label.text = text.str();
            arr.markers.push_back(label);
        } else {
            visualization_msgs::Marker actual;
            actual.header = m.header;
            actual.ns = actual_ns;
            actual.id = marker_id;
            actual.type = visualization_msgs::Marker::SPHERE;
            actual.action = visualization_msgs::Marker::ADD;
            actual.pose.position.x = c.conflict_x;
            actual.pose.position.y = c.conflict_y;
            actual.pose.position.z = 0.060;
            actual.pose.orientation.w = 1.0;
            actual.scale.x = 0.055;
            actual.scale.y = 0.055;
            actual.scale.z = 0.055;
            actual.color = rgba(1.00f, 0.10f, 0.08f, 1.0f);
            arr.markers.push_back(actual);

            visualization_msgs::Marker label;
            label.header = m.header;
            label.ns = conflict_label_ns;
            label.id = marker_id;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position.x = c.conflict_x;
            label.pose.position.y = c.conflict_y;
            label.pose.position.z = 0.16;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.070;
            label.color = rgba(1.00f, 0.80f, 0.25f, 1.0f);
            std::ostringstream text;
            text << std::fixed << std::setprecision(2)
                 << "control OBB overlap center V" << c.vehicle_a
                 << "-V" << c.vehicle_b
                 << " type=CROSSING/OPPOSING t=" << c.t << "s"
                 << "\norange = search AABB";
            label.text = text.str();
            arr.markers.push_back(label);
        }
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
    for (int stale = same_id;
         stale < last_same_direction_conflict_marker_count_; ++stale) {
        deleteMarker(following_relation_ns, stale);
        deleteMarker(following_label_ns, stale);
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
    for (int stale = mutual_id;
         stale < last_crossing_opposing_conflict_marker_count_; ++stale) {
        deleteMarker(actual_ns, stale);
        deleteMarker(conflict_label_ns, stale);
    }
    last_same_direction_conflict_marker_count_ = same_id;
    last_crossing_opposing_conflict_marker_count_ = mutual_id;
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
    const std::vector<ConflictMarker>& conflicts) const {
    ++publish_seq_;
    visualization_msgs::MarkerArray arr;
    addVisitedSlotMarkers(arr, visited_slots);
    addA1PickupDefinitionMarkers(arr);
    addOriginAxes(arr);  // 地图原点+XY正方向(标定核对用)
    for (const VehicleAgent& v : vehicles) {
        addPathMarker(arr, v);
        if (v.mode == VehicleMode::NEED_TASK || v.track.empty()) continue;
        addBodyMarker(arr, v);
        addArrowMarker(arr, v);
        addLabelMarker(arr, v);
    }
    addConflictMarkers(arr, conflicts);
    pub_.publish(arr);
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
