#include <gtest/gtest.h>

#include "Geometry.hpp"

using namespace r3d;

namespace {
void expectVec(const Vec3& a, const Vec3& b, double tol = 1e-9) {
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}
}  // namespace

TEST(Geometry, LocalToWorldYaw90) {
    Pose frame{{1, 2, 0}, {0, 0, kPi / 2}};
    expectVec(localToWorld(frame, {1, 0, 0}), {1, 3, 0});  // local +x points along world +y
}

TEST(Geometry, WorldToLocalIsInverse) {
    Pose frame{{3, -1, 0.5}, {0.1, -0.2, 0.7}};
    Vec3 p{4, 5, 6};
    expectVec(worldToLocal(frame, localToWorld(frame, p)), p);
    expectVec(localToWorld(frame, worldToLocal(frame, p)), p);
}

TEST(Geometry, RotationIsOrthonormal) {
    Mat3 R = Mat3::fromEuler(0.3, -0.4, 1.2);
    Mat3 I = R * R.transposed();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) EXPECT_NEAR(I(r, c), r == c ? 1.0 : 0.0, 1e-12);
}

TEST(Geometry, LidarToRobotToWorldChain) {
    Pose lidarInRobot{{0, 0, 0.5}, {0, 0, 0}};
    Pose robotInWorld{{2, 1, 0}, {0, 0, kPi / 2}};
    Transform lidarToWorld = Transform::fromPose(robotInWorld) * Transform::fromPose(lidarInRobot);
    Vec3 pLidar{2, 0, 0};  // 2 m in front of the sensor
    // Step by step: robot frame (2, 0, 0.5) -> world (2, 1+2, 0.5)
    expectVec(lidarToWorld.toParent(pLidar), {2, 3, 0.5});
    expectVec(lidarToWorld.toLocal(lidarToWorld.toParent(pLidar)), pLidar);
    expectVec(lidarToWorld.inverse().toParent({2, 3, 0.5}), pLidar);
}

TEST(Geometry, WrapAngle) {
    EXPECT_NEAR(wrapAngle(3 * kPi / 2), -kPi / 2, 1e-12);
    EXPECT_NEAR(wrapAngle(-3 * kPi / 2), kPi / 2, 1e-12);
}
