#pragma once
// Basic 3D math: vectors, rotation matrices, rigid transforms, ray/box tests.
// Convention: right-handed, Z up. Euler angles are (roll, pitch, yaw) about
// (X, Y, Z) and compose as R = Rz(yaw) * Ry(pitch) * Rx(roll).
#include <array>
#include <cmath>
#include <numbers>
#include <optional>

namespace r3d {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double deg2rad(double d) { return d * kPi / 180.0; }
double wrapAngle(double a);  // wrap to (-pi, pi]

struct Vec3 {
    double x{0}, y{0}, z{0};
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double norm() const { return std::sqrt(dot(*this)); }
    double normXY() const { return std::hypot(x, y); }
    Vec3 normalized() const {
        double n = norm();
        return n > 0 ? *this * (1.0 / n) : Vec3{};
    }
};
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// Position + orientation (roll, pitch, yaw). The robot is planar, so only
// position.x/y and rotation.z (yaw) change while driving.
struct Pose {
    Vec3 position;
    Vec3 rotation;
};

struct Mat3 {
    std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};  // row-major
    static Mat3 identity() { return {}; }
    static Mat3 fromEuler(double roll, double pitch, double yaw);
    double operator()(int r, int c) const { return m[r * 3 + c]; }
    double& operator()(int r, int c) { return m[r * 3 + c]; }
    Mat3 transposed() const;
    Vec3 operator*(const Vec3& v) const;
    Mat3 operator*(const Mat3& o) const;
};

// Rigid transform "child -> parent": p_parent = R * p_child + t.
struct Transform {
    Mat3 R;
    Vec3 t;
    static Transform fromPose(const Pose& pose);
    Vec3 toParent(const Vec3& p) const { return R * p + t; }                  // local -> parent
    Vec3 toLocal(const Vec3& p) const { return R.transposed() * (p - t); }    // parent -> local
    Transform operator*(const Transform& o) const { return {R * o.R, R * o.t + t}; }
    Transform inverse() const;
};

// Convenience wrappers: `frame` is the pose of a local frame expressed in the world.
Vec3 localToWorld(const Pose& frame, const Vec3& pLocal);
Vec3 worldToLocal(const Pose& frame, const Vec3& pWorld);

struct Ray {
    Vec3 origin;
    Vec3 dir;  // unit length
};

struct AABB {
    Vec3 min, max;
    Vec3 center() const { return (min + max) * 0.5; }
    Vec3 size() const { return max - min; }
};

// Slab method. Returns distance along the ray to the first entry point,
// or nullopt if the ray misses, starts inside the box, or the hit is beyond tMax.
std::optional<double> intersectRayAABB(const Ray& ray, const AABB& box, double tMax);
// Intersection with the horizontal plane z = 0 (ground), for rays heading downward.
std::optional<double> intersectRayGround(const Ray& ray, double tMax);

}  // namespace r3d
