#include "Lidar.hpp"

namespace r3d {

LidarScan Lidar::scan(const Environment& env, const Pose& robotPose) const {
    LidarScan s;
    // LiDAR -> Robot -> World
    const Transform lidarToRobot = Transform::fromPose(mount_);
    const Transform robotToWorld = Transform::fromPose(robotPose);
    s.lidarToWorld = robotToWorld * lidarToRobot;

    for (double elDeg : cfg_.verticalAnglesDeg) {
        const double el = deg2rad(elDeg);
        for (int i = 0; i < cfg_.horizontalRays; ++i) {
            const double az = 2.0 * kPi * i / cfg_.horizontalRays;
            const Vec3 dirLocal{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
            const Ray ray{s.lidarToWorld.t, s.lidarToWorld.R * dirLocal};
            const RayHit hit = env.raycast(ray, cfg_.maxRange);
            s.returns.push_back({dirLocal * hit.t, hit.t, hit.type});
        }
    }
    return s;
}

std::vector<Vec3> detectObstaclePoints(const LidarScan& scan, double minHeight, double maxHeight) {
    std::vector<Vec3> pts;
    for (const auto& r : scan.returns) {
        if (r.type != HitType::Obstacle) continue;
        const Vec3 p = scan.pointWorld(r);
        if (p.z >= minHeight && p.z <= maxHeight) pts.push_back(p);
    }
    return pts;
}

}  // namespace r3d
