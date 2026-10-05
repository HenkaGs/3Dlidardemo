#pragma once
#include <utility>
#include <vector>

#include "Environment.hpp"
#include "Geometry.hpp"

namespace r3d {

struct LidarConfig {
    int horizontalRays{72};
    std::vector<double> verticalAnglesDeg{-20, -10, 0, 10};
    double maxRange{8.0};
};

struct LidarReturn {
    Vec3 pointLidar;  // hit point in the LiDAR frame (range * ray direction)
    double range{0};
    HitType type{HitType::None};
};

struct LidarScan {
    Transform lidarToWorld;  // = robotToWorld * lidarToRobot
    std::vector<LidarReturn> returns;
    Vec3 originWorld() const { return lidarToWorld.t; }
    Vec3 pointWorld(const LidarReturn& r) const { return lidarToWorld.toParent(r.pointLidar); }
};

class Lidar {
public:
    explicit Lidar(LidarConfig cfg = {}, Pose mountOnRobot = {{0, 0, 0.5}, {0, 0, 0}})
        : cfg_(std::move(cfg)), mount_(mountOnRobot) {}
    const LidarConfig& config() const { return cfg_; }

    // Cast every ray from the sensor at the given robot pose.
    LidarScan scan(const Environment& env, const Pose& robotPose) const;

private:
    LidarConfig cfg_;
    Pose mount_;  // LiDAR frame expressed in the robot frame
};

// Obstacle detection: world-frame points of obstacle returns whose height lies in
// [minHeight, maxHeight] (ground returns are discarded).
std::vector<Vec3> detectObstaclePoints(const LidarScan& scan, double minHeight = 0.05,
                                       double maxHeight = 2.0);

}  // namespace r3d
