#include "Environment.hpp"

namespace r3d {

void Environment::addBox(double cx, double cy, double sx, double sy, double h, bool known) {
    obstacles.push_back({{{cx - sx / 2, cy - sy / 2, 0}, {cx + sx / 2, cy + sy / 2, h}}, known});
}

Environment Environment::makeDemo() {
    Environment e;
    // Boundary walls just outside the playable area.
    const double w = 0.3, h = 1.5;
    e.addBox(0, e.yMin - w / 2, 12 + 2 * w, w, h);
    e.addBox(0, e.yMax + w / 2, 12 + 2 * w, w, h);
    e.addBox(e.xMin - w / 2, 0, w, 12, h);
    e.addBox(e.xMax + w / 2, 0, w, 12, h);
    // Known obstacles (prior map).
    e.addBox(-0.25, -2.25, 0.5, 7.5, 1.5);   // long wall forcing a detour
    e.addBox(-3.0, 2.5, 1.2, 1.2, 1.0);      // box
    e.addBox(2.5, -3.0, 1.0, 1.0, 0.6);      // rock
    // Unknown obstacles: not in the prior map, found by LiDAR -> replanning.
    e.addBox(2.5, 3.3, 1.6, 1.6, 1.2, false);
    e.addBox(4.2, 0.8, 1.0, 1.0, 0.8, false);
    return e;
}

RayHit Environment::raycast(const Ray& ray, double maxRange) const {
    RayHit best{maxRange, HitType::None};
    for (const auto& ob : obstacles) {
        if (auto t = intersectRayAABB(ray, ob.box, best.t); t && *t < best.t) best = {*t, HitType::Obstacle};
    }
    if (auto t = intersectRayGround(ray, best.t); t && *t < best.t) best = {*t, HitType::Ground};
    return best;
}

bool Environment::occupiedXY(double x, double y) const {
    for (const auto& ob : obstacles)
        if (x >= ob.box.min.x && x <= ob.box.max.x && y >= ob.box.min.y && y <= ob.box.max.y) return true;
    return false;
}

}  // namespace r3d
