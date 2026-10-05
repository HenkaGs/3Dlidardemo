#include "Robot.hpp"

namespace r3d {

void Robot::drive(double v, double w, double dt) {
    pose_.rotation.z = wrapAngle(pose_.rotation.z + w * dt);
    pose_.position.x += v * std::cos(pose_.rotation.z) * dt;
    pose_.position.y += v * std::sin(pose_.rotation.z) * dt;
}

}  // namespace r3d
