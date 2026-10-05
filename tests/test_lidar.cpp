#include <gtest/gtest.h>

#include <algorithm>

#include "Lidar.hpp"

using namespace r3d;

namespace {
const AABB kBox{{2, -1, 0}, {3, 1, 2}};
}

TEST(RayBox, HitsFrontFace) {
    auto t = intersectRayAABB({{0, 0, 1}, {1, 0, 0}}, kBox, 100);
    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(*t, 2.0, 1e-9);
}

TEST(RayBox, MissesAndRange) {
    EXPECT_FALSE(intersectRayAABB({{0, 5, 1}, {1, 0, 0}}, kBox, 100).has_value());   // passes beside
    EXPECT_FALSE(intersectRayAABB({{0, 0, 1}, {-1, 0, 0}}, kBox, 100).has_value());  // points away
    EXPECT_FALSE(intersectRayAABB({{0, 0, 1}, {1, 0, 0}}, kBox, 1.5).has_value());   // out of range
}

TEST(RayBox, AngledRay) {
    Vec3 d = Vec3{1, 0.25, 0}.normalized();  // reaches the x = 2 face at y = 0.5 (inside the face)
    auto t = intersectRayAABB({{0, 0, 1}, d}, kBox, 100);
    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(*t, 2.0 * std::sqrt(1.0625), 1e-9);
    Vec3 d2 = Vec3{1, 1, 0}.normalized();  // reaches x = 2 at y = 2, beyond the box
    EXPECT_FALSE(intersectRayAABB({{0, 0, 1}, d2}, kBox, 100).has_value());
}

TEST(Lidar, DetectsObstacleAheadAtCorrectRange) {
    Environment env;
    env.addBox(3.0, 0.0, 1.0, 4.0, 2.0);  // wall face at x = 2.5
    Lidar lidar;                          // mounted 0.5 m above the robot origin
    LidarScan scan = lidar.scan(env, Pose{{0, 0, 0}, {0, 0, 0}});

    // Ray 0 of the 0-degree layer points straight along +x.
    const size_t nH = lidar.config().horizontalRays;
    const LidarReturn& fwd = scan.returns[2 * nH];  // layers: -20, -10, 0, 10
    EXPECT_EQ(fwd.type, HitType::Obstacle);
    EXPECT_NEAR(fwd.range, 2.5, 1e-9);
    Vec3 w = scan.pointWorld(fwd);
    EXPECT_NEAR(w.x, 2.5, 1e-9);
    EXPECT_NEAR(w.z, 0.5, 1e-9);

    EXPECT_FALSE(detectObstaclePoints(scan).empty());
}

TEST(Lidar, EmptyWorldOnlySeesGround) {
    Environment env;
    Lidar lidar;
    LidarScan scan = lidar.scan(env, Pose{});
    for (const auto& r : scan.returns) EXPECT_NE(r.type, HitType::Obstacle);
    EXPECT_TRUE(detectObstaclePoints(scan).empty());
    bool anyGround = std::any_of(scan.returns.begin(), scan.returns.end(),
                                 [](const LidarReturn& r) { return r.type == HitType::Ground; });
    EXPECT_TRUE(anyGround);
}

TEST(Lidar, RobotYawRotatesScan) {
    Environment env;
    env.addBox(0.0, 3.0, 4.0, 1.0, 2.0);  // wall face at y = 2.5, to the robot's left
    Lidar lidar;
    // Facing +y (yaw 90 deg): the wall is now straight ahead, so ray 0 of the 0-degree layer hits at 2.5 m.
    LidarScan scan = lidar.scan(env, Pose{{0, 0, 0}, {0, 0, kPi / 2}});
    const size_t nH = lidar.config().horizontalRays;
    EXPECT_NEAR(scan.returns[2 * nH].range, 2.5, 1e-9);
}
