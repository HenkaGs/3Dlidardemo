#include "Simulation.hpp"

namespace r3d {
namespace {
constexpr double kResolution = 0.25;
constexpr double kInflation = 0.35 + 0.10;  // robot radius + safety margin
}  // namespace

Simulation::Simulation()
    : env_(Environment::makeDemo()),
      robot_(),
      lidar_(),
      grid_(env_.xMin, env_.yMin, static_cast<int>((env_.xMax - env_.xMin) / kResolution),
            static_cast<int>((env_.yMax - env_.yMin) / kResolution), kResolution, kInflation) {
    reset();
}

void Simulation::reset() {
    env_ = Environment::makeDemo();
    robot_ = Robot(Pose{env_.start, {0, 0, kPi / 4}});
    grid_ = OccupancyGrid(env_.xMin, env_.yMin, static_cast<int>((env_.xMax - env_.xMin) / kResolution),
                          static_cast<int>((env_.yMax - env_.yMin) / kResolution), kResolution, kInflation);
    for (const auto& ob : env_.obstacles)
        if (ob.known) grid_.rasterizeBox(ob.box);  // prior map only
    controller_ = Controller();
    scan_ = {};
    replans_ = 0;
    time_ = 0;
    status_ = plan() ? SimStatus::Navigating : SimStatus::NoPath;
}

bool Simulation::plan() {
    const Cell s = grid_.worldToCell(robot_.pose().position.x, robot_.pose().position.y);
    const Cell g = grid_.worldToCell(env_.goal.x, env_.goal.y);
    auto path = planPath(grid_, s, g);
    if (!path) {
        controller_.stop();
        return false;
    }
    auto wp = cellsToWaypoints(grid_, *path);
    if (wp.size() > 1) wp.erase(wp.begin());  // robot is already in the first cell
    wp.back() = env_.goal;                    // finish exactly on the goal
    controller_.setWaypoints(std::move(wp));
    return true;
}

// Sample the remaining route (robot -> next waypoint -> ...) against the inflated grid.
bool Simulation::pathBlocked() const {
    const auto& wp = controller_.waypoints();
    Vec3 a = robot_.pose().position;
    for (size_t i = controller_.nextIndex(); i < wp.size(); ++i) {
        const Vec3 b = wp[i];
        const double len = (b - a).normXY();
        const int n = std::max(1, static_cast<int>(len / (kResolution * 0.5)));
        for (int k = 0; k <= n; ++k) {
            const Vec3 p = a + (b - a) * (static_cast<double>(k) / n);
            if ((p - robot_.pose().position).normXY() < 0.3) continue;  // ignore our own footprint
            if (grid_.isBlocked(grid_.worldToCell(p.x, p.y))) return true;
        }
        a = b;
    }
    return false;
}

void Simulation::step(double dt) {
    if (status_ == SimStatus::GoalReached) return;
    time_ += dt;

    // Perception: LiDAR -> point cloud -> obstacle cells
    scan_ = lidar_.scan(env_, robot_.pose());
    int newCells = 0;
    const Vec3 origin = scan_.originWorld();
    for (const Vec3& p : detectObstaclePoints(scan_)) {
        // Nudge 5 cm along the ray so the point lands inside the obstacle's cell, not on its face.
        const Vec3 q = p + (p - origin).normalized() * 0.05;
        newCells += grid_.setOccupied(grid_.worldToCell(q.x, q.y));
    }

    // Replan if the map changed and the current route is no longer safe (or we had no route).
    if (newCells > 0 && (status_ == SimStatus::NoPath || pathBlocked())) {
        ++replans_;
        status_ = plan() ? SimStatus::Navigating : SimStatus::NoPath;
    }

    if (status_ == SimStatus::Navigating) {
        controller_.update(robot_, dt);
        if (controller_.state() == ControllerState::GoalReached) status_ = SimStatus::GoalReached;
    }
}

}  // namespace r3d
