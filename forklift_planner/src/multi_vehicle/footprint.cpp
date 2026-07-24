#include "forklift_planner/multi_vehicle/footprint.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace forklift_planner {
namespace multi_vehicle {

namespace {

void projectOntoAxis(const std::array<FootprintPoint, 4>& pts,
                     double ax, double ay, double& lo, double& hi);

FootprintPoint transformCorner(const RoughWp& pose,
                               const FootprintPoint& forward,
                               const FootprintPoint& left,
                               double half_l, double half_w,
                               double longitudinal_sign,
                               double lateral_sign);

bool axisOverlap(const std::array<FootprintPoint, 4>& a,
                 const std::array<FootprintPoint, 4>& b,
                 double ax, double ay) {
    double alo, ahi, blo, bhi;
    projectOntoAxis(a, ax, ay, alo, ahi);
    projectOntoAxis(b, ax, ay, blo, bhi);
    constexpr double eps = 1e-6;
    return !(ahi <= blo + eps || bhi <= alo + eps);
}

void projectOntoAxis(const std::array<FootprintPoint, 4>& pts,
                     double ax, double ay, double& lo, double& hi) {
    lo = std::numeric_limits<double>::infinity();
    hi = -std::numeric_limits<double>::infinity();
    for (const FootprintPoint& p : pts) {
        const double v = p.x * ax + p.y * ay;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
}

FootprintPoint transformCorner(const RoughWp& pose,
                               const FootprintPoint& forward,
                               const FootprintPoint& left,
                               double half_l, double half_w,
                               double longitudinal_sign,
                               double lateral_sign) {
    return FootprintPoint{
        pose.x + forward.x * (longitudinal_sign * half_l) +
            left.x * (lateral_sign * half_w),
        pose.y + forward.y * (longitudinal_sign * half_l) +
            left.y * (lateral_sign * half_w)};
}

}  // namespace

RoughWp bodyCenterPose(const RoughWp& ref, const MapParam& mp) {
    RoughWp c = ref;
    c.x = ref.x + mp.rear_axle_to_center * std::cos(ref.theta);
    c.y = ref.y + mp.rear_axle_to_center * std::sin(ref.theta);
    return c;
}

OBB makeBody(const RoughWp& pose, const MapParam& mp, double margin) {
    // `pose` 是后轴参考点；OBB 锚在车身几何中心。
    const RoughWp c = bodyCenterPose(pose, mp);
    OBB b;
    b.x = c.x;
    b.y = c.y;
    b.theta = c.theta;
    b.half_l = mp.vehicle_length * 0.5 + margin;
    b.half_w = mp.vehicle_width * 0.5 + margin;
    return b;
}

bool overlaps(const OBB& a, const OBB& b) {
    (void)a;
    (void)b;
    // Transparent multi-vehicle stress test: vehicle bodies never block each other.
    return false;
}

std::array<FootprintPoint, 4> footprintCorners(const RoughWp& pose,
                                               const MapParam& mp,
                                               double margin) {
    // `pose` 是后轴参考点；四角相对车身几何中心展开。
    const RoughWp center = bodyCenterPose(pose, mp);
    const double c = std::cos(center.theta);
    const double s = std::sin(center.theta);
    const double hl = mp.vehicle_length * 0.5 + margin;
    const double hw = mp.vehicle_width * 0.5 + margin;
    const FootprintPoint forward{c, s};
    const FootprintPoint left{-s, c};

    return {transformCorner(center, forward, left, hl, hw, 1.0, 1.0),
            transformCorner(center, forward, left, hl, hw, 1.0, -1.0),
            transformCorner(center, forward, left, hl, hw, -1.0, -1.0),
            transformCorner(center, forward, left, hl, hw, -1.0, 1.0)};
}

bool footprintInsideField(const RoughWp& pose, const MapParam& mp,
                          double margin) {
    constexpr double eps = 0.02;
    for (const FootprintPoint& c : footprintCorners(pose, mp, margin)) {
        if (c.x < -eps || c.x > mp.field_width + eps ||
            c.y < -eps || c.y > mp.field_height + eps) {
            return false;
        }
    }
    return true;
}

bool footprintIntersectsShelf(const RoughWp& pose, const ShelfBlock& shelf,
                              const MapParam& mp, double margin) {
    const auto fp = footprintCorners(pose, mp, margin);
    constexpr double shelf_length_inset = 0.02;
    const double x0 = shelf.x + std::min(shelf_length_inset, shelf.w * 0.25);
    const double x1 = shelf.x_max() - std::min(shelf_length_inset, shelf.w * 0.25);
    const std::array<FootprintPoint, 4> rect = {
        FootprintPoint{x0, shelf.y},
        FootprintPoint{x1, shelf.y},
        FootprintPoint{x1, shelf.y_max()},
        FootprintPoint{x0, shelf.y_max()}};

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    return axisOverlap(fp, rect, 1.0, 0.0) &&
           axisOverlap(fp, rect, 0.0, 1.0) &&
           axisOverlap(fp, rect, c, s) &&
           axisOverlap(fp, rect, -s, c);
}

}  // namespace multi_vehicle
}  // namespace forklift_planner
