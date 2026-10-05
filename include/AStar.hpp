#pragma once
#include <cstdint>
#include <optional>
#include <vector>

#include "Geometry.hpp"

namespace r3d {

struct Cell {
    int x{0}, y{0};
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

// 2.5D occupancy grid: the world is flattened onto the X/Y plane.
// 0 = free, 1 = obstacle. Obstacles are inflated by the robot radius (+margin)
// so the planner can treat the robot as a point.
class OccupancyGrid {
public:
    OccupancyGrid(double xMin, double yMin, int width, int height, double resolution,
                  double inflationRadius);

    int width() const { return w_; }
    int height() const { return h_; }
    double resolution() const { return res_; }

    Cell worldToCell(double x, double y) const;
    Vec3 cellCenter(Cell c) const;  // z = 0
    bool inBounds(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < w_ && c.y < h_; }

    bool isOccupied(Cell c) const;  // raw obstacle (0/1)
    bool isBlocked(Cell c) const;   // inflated obstacle or outside the grid
    bool setOccupied(Cell c);       // returns true if the cell was newly marked
    int rasterizeBox(const AABB& box);  // marks every cell the footprint overlaps; returns # new

private:
    double x0_, y0_, res_;
    int w_, h_, inflateCells_;
    std::vector<std::uint8_t> raw_, inflated_;
};

// A* on an 8-connected grid, Euclidean heuristic, no diagonal corner cutting.
// The start cell may lie inside the inflated zone (robot already close to an
// obstacle); the goal may not. Returns nullopt if no path exists.
std::optional<std::vector<Cell>> planPath(const OccupancyGrid& grid, Cell start, Cell goal);

// Cell path -> world waypoints, merging collinear steps.
std::vector<Vec3> cellsToWaypoints(const OccupancyGrid& grid, const std::vector<Cell>& path);

}  // namespace r3d
