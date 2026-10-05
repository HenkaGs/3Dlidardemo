#include "Geometry.hpp"

#include <algorithm>
#include <limits>

namespace r3d {

double wrapAngle(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0) a += 2.0 * kPi;
    return a - kPi;
}

Mat3 Mat3::fromEuler(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    Mat3 R;
    R(0, 0) = cy * cp; R(0, 1) = cy * sp * sr - sy * cr; R(0, 2) = cy * sp * cr + sy * sr;
    R(1, 0) = sy * cp; R(1, 1) = sy * sp * sr + cy * cr; R(1, 2) = sy * sp * cr - cy * sr;
    R(2, 0) = -sp;     R(2, 1) = cp * sr;                R(2, 2) = cp * cr;
    return R;
}

Mat3 Mat3::transposed() const {
    Mat3 T;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) T(r, c) = (*this)(c, r);
    return T;
}

Vec3 Mat3::operator*(const Vec3& v) const {
    return {(*this)(0, 0) * v.x + (*this)(0, 1) * v.y + (*this)(0, 2) * v.z,
            (*this)(1, 0) * v.x + (*this)(1, 1) * v.y + (*this)(1, 2) * v.z,
            (*this)(2, 0) * v.x + (*this)(2, 1) * v.y + (*this)(2, 2) * v.z};
}

Mat3 Mat3::operator*(const Mat3& o) const {
    Mat3 out;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += (*this)(r, k) * o(k, c);
            out(r, c) = s;
        }
    return out;
}

Transform Transform::fromPose(const Pose& pose) {
    return {Mat3::fromEuler(pose.rotation.x, pose.rotation.y, pose.rotation.z), pose.position};
}

Transform Transform::inverse() const {
    Mat3 Rt = R.transposed();
    return {Rt, -(Rt * t)};
}

Vec3 localToWorld(const Pose& frame, const Vec3& p) { return Transform::fromPose(frame).toParent(p); }
Vec3 worldToLocal(const Pose& frame, const Vec3& p) { return Transform::fromPose(frame).toLocal(p); }

std::optional<double> intersectRayAABB(const Ray& ray, const AABB& box, double tMax) {
    double tNear = -std::numeric_limits<double>::infinity();
    double tFar = std::numeric_limits<double>::infinity();
    const double o[3] = {ray.origin.x, ray.origin.y, ray.origin.z};
    const double d[3] = {ray.dir.x, ray.dir.y, ray.dir.z};
    const double lo[3] = {box.min.x, box.min.y, box.min.z};
    const double hi[3] = {box.max.x, box.max.y, box.max.z};
    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-12) {
            if (o[i] < lo[i] || o[i] > hi[i]) return std::nullopt;  // parallel and outside slab
            continue;
        }
        double t1 = (lo[i] - o[i]) / d[i];
        double t2 = (hi[i] - o[i]) / d[i];
        if (t1 > t2) std::swap(t1, t2);
        tNear = std::max(tNear, t1);
        tFar = std::min(tFar, t2);
        if (tNear > tFar) return std::nullopt;
    }
    if (tNear < 0 || tNear > tMax) return std::nullopt;
    return tNear;
}

std::optional<double> intersectRayGround(const Ray& ray, double tMax) {
    if (ray.dir.z >= -1e-12 || ray.origin.z < 0) return std::nullopt;
    double t = -ray.origin.z / ray.dir.z;
    if (t > tMax) return std::nullopt;
    return t;
}

}  // namespace r3d
