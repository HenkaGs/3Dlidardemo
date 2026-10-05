#pragma once
#include <vector>

#include "Robot.hpp"

namespace r3d {

struct ControllerConfig {
    double maxLinearSpeed{1.5};    // m/s
    double maxAngularSpeed{3.0};   // rad/s
    double turnGain{4.0};
    double turnInPlaceAngle{0.6};  // rad: above this heading error, rotate without moving
    double waypointTolerance{0.25};
    double goalTolerance{0.12};
};

enum class ControllerState { Idle, Following, GoalReached };

// Waypoint follower: rotate toward the next waypoint, drive forward, advance when close.
class Controller {
public:
    explicit Controller(ControllerConfig cfg = {}) : cfg_(cfg) {}

    void setWaypoints(std::vector<Vec3> wp);  // restarts from the first waypoint
    void stop();
    void update(Robot& robot, double dt);

    ControllerState state() const { return state_; }
    const std::vector<Vec3>& waypoints() const { return wp_; }
    size_t nextIndex() const { return idx_; }

private:
    ControllerConfig cfg_;
    std::vector<Vec3> wp_;
    size_t idx_{0};
    ControllerState state_{ControllerState::Idle};
};

}  // namespace r3d
