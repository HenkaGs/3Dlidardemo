#pragma once
#include "Geometry.hpp"

namespace r3d {

// Planar kinematic robot: x/y translation + yaw. No dynamics.
class Robot {
public:
    explicit Robot(Pose pose = {}, double radius = 0.35) : pose_(pose), radius_(radius) {}
    const Pose& pose() const { return pose_; }
    double yaw() const { return pose_.rotation.z; }
    double radius() const { return radius_; }
    void setPose(const Pose& p) { pose_ = p; }

    // Drive with forward speed v [m/s] and yaw rate w [rad/s] for dt seconds.
    void drive(double v, double w, double dt);

private:
    Pose pose_;
    double radius_;
};

}  // namespace r3d
