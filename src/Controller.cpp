#include "Controller.hpp"

#include <algorithm>

namespace r3d {

void Controller::setWaypoints(std::vector<Vec3> wp) {
    wp_ = std::move(wp);
    idx_ = 0;
    state_ = wp_.empty() ? ControllerState::Idle : ControllerState::Following;
}

void Controller::stop() {
    wp_.clear();
    idx_ = 0;
    state_ = ControllerState::Idle;
}

void Controller::update(Robot& robot, double dt) {
    if (state_ != ControllerState::Following) return;

    // 1. select next waypoint (advance past any we are already close to)
    while (idx_ < wp_.size()) {
        const bool last = idx_ + 1 == wp_.size();
        const Vec3 d = wp_[idx_] - robot.pose().position;
        if (d.normXY() > (last ? cfg_.goalTolerance : cfg_.waypointTolerance)) break;
        ++idx_;
    }
    if (idx_ >= wp_.size()) {  // 6. goal reached
        state_ = ControllerState::GoalReached;
        return;
    }

    // 2. direction to the waypoint
    const Vec3 d = wp_[idx_] - robot.pose().position;
    const double dist = d.normXY();
    const double err = wrapAngle(std::atan2(d.y, d.x) - robot.yaw());

    // 3. rotate toward it
    const double w = std::clamp(cfg_.turnGain * err, -cfg_.maxAngularSpeed, cfg_.maxAngularSpeed);
    // 4. move forward (only once roughly facing it; never overshoot this step)
    double v = 0.0;
    if (std::abs(err) < cfg_.turnInPlaceAngle)
        v = std::min(cfg_.maxLinearSpeed * std::cos(err), dist / dt);
    robot.drive(v, w, dt);
}

}  // namespace r3d
