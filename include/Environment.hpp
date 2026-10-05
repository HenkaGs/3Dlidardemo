#pragma once
#include <optional>
#include <vector>

#include "Geometry.hpp"

namespace r3d {

enum class HitType { None, Ground, Obstacle };
struct RayHit {
    double t;
    HitType type;
};

struct Obstacle {
    AABB box;
    // Known obstacles are in the robot's prior map. Unknown ones can only be
    // discovered by the LiDAR, which is what triggers replanning.
    bool known{true};
};

// Ground plane (z = 0) inside [xMin,xMax] x [yMin,yMax], plus box obstacles.
class Environment {
public:
    double xMin{-6}, xMax{6}, yMin{-6}, yMax{6};
    std::vector<Obstacle> obstacles;
    Vec3 start{-5, -5, 0};
    Vec3 goal{5, 5, 0};

    static Environment makeDemo();
    void addBox(double cx, double cy, double sx, double sy, double height, bool known = true);

    // Closest thing hit by the ray within maxRange (t == maxRange, type None if nothing).
    RayHit raycast(const Ray& ray, double maxRange) const;
    // True if (x, y) lies inside any obstacle footprint.
    bool occupiedXY(double x, double y) const;
};

}  // namespace r3d
