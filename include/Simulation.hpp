#pragma once
#include "AStar.hpp"
#include "Controller.hpp"
#include "Environment.hpp"
#include "Lidar.hpp"
#include "Robot.hpp"

namespace r3d {

enum class SimStatus { Navigating, GoalReached, NoPath };

// Ties the pipeline together (no rendering):
// Environment -> LiDAR -> obstacle points -> occupancy grid -> A* -> waypoints -> controller -> robot
class Simulation {
public:
    Simulation();
    void reset();
    void step(double dt);

    const Environment& environment() const { return env_; }
    const Robot& robot() const { return robot_; }
    const OccupancyGrid& grid() const { return grid_; }
    const LidarScan& lastScan() const { return scan_; }
    const Controller& controller() const { return controller_; }
    SimStatus status() const { return status_; }
    int replanCount() const { return replans_; }
    double elapsed() const { return time_; }

private:
    bool plan();            // A* from the robot's cell to the goal; installs waypoints
    bool pathBlocked() const;

    Environment env_;
    Robot robot_;
    Lidar lidar_;
    OccupancyGrid grid_;
    Controller controller_;
    LidarScan scan_;
    SimStatus status_{SimStatus::Navigating};
    int replans_{0};
    double time_{0};
};

}  // namespace r3d
