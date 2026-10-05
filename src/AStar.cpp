#include "AStar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace r3d {

OccupancyGrid::OccupancyGrid(double xMin, double yMin, int width, int height, double resolution,
                             double inflationRadius)
    : x0_(xMin), y0_(yMin), res_(resolution), w_(width), h_(height),
      inflateCells_(static_cast<int>(std::ceil(inflationRadius / resolution))),
      raw_(static_cast<size_t>(width) * height, 0), inflated_(raw_.size(), 0) {}

Cell OccupancyGrid::worldToCell(double x, double y) const {
    return {static_cast<int>(std::floor((x - x0_) / res_)), static_cast<int>(std::floor((y - y0_) / res_))};
}

Vec3 OccupancyGrid::cellCenter(Cell c) const {
    return {x0_ + (c.x + 0.5) * res_, y0_ + (c.y + 0.5) * res_, 0.0};
}

bool OccupancyGrid::isOccupied(Cell c) const { return inBounds(c) && raw_[c.y * w_ + c.x]; }
bool OccupancyGrid::isBlocked(Cell c) const { return !inBounds(c) || inflated_[c.y * w_ + c.x]; }

bool OccupancyGrid::setOccupied(Cell c) {
    if (!inBounds(c) || raw_[c.y * w_ + c.x]) return false;
    raw_[c.y * w_ + c.x] = 1;
    // Stamp a disc of radius inflateCells_ into the inflated layer.
    for (int dy = -inflateCells_; dy <= inflateCells_; ++dy)
        for (int dx = -inflateCells_; dx <= inflateCells_; ++dx) {
            if (dx * dx + dy * dy > inflateCells_ * inflateCells_) continue;
            Cell n{c.x + dx, c.y + dy};
            if (inBounds(n)) inflated_[n.y * w_ + n.x] = 1;
        }
    return true;
}

int OccupancyGrid::rasterizeBox(const AABB& b) {
    Cell lo = worldToCell(b.min.x, b.min.y), hi = worldToCell(b.max.x, b.max.y);
    int added = 0;
    for (int y = std::max(lo.y, 0); y <= std::min(hi.y, h_ - 1); ++y)
        for (int x = std::max(lo.x, 0); x <= std::min(hi.x, w_ - 1); ++x) added += setOccupied({x, y});
    return added;
}

std::optional<std::vector<Cell>> planPath(const OccupancyGrid& g, Cell start, Cell goal) {
    if (!g.inBounds(start) || !g.inBounds(goal) || g.isBlocked(goal)) return std::nullopt;

    const int W = g.width();
    const auto idx = [W](Cell c) { return c.y * W + c.x; };
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> gScore(static_cast<size_t>(W) * g.height(), inf);
    std::vector<int> parent(gScore.size(), -1);
    std::vector<char> closed(gScore.size(), 0);

    const auto h = [&](Cell c) { return std::hypot(c.x - goal.x, c.y - goal.y); };  // Euclidean
    using QE = std::pair<double, int>;  // (f, cell index)
    std::priority_queue<QE, std::vector<QE>, std::greater<>> open;
    gScore[idx(start)] = 0;
    open.push({h(start), idx(start)});

    while (!open.empty()) {
        const int i = open.top().second;
        open.pop();
        if (closed[i]) continue;
        closed[i] = 1;
        const Cell c{i % W, i / W};

        if (c == goal) {
            std::vector<Cell> path;
            for (int k = i; k != -1; k = parent[k]) path.push_back({k % W, k / W});
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const Cell n{c.x + dx, c.y + dy};
                if (g.isBlocked(n) || closed[idx(n)]) continue;
                const bool diagonal = dx != 0 && dy != 0;
                if (diagonal && (g.isBlocked({c.x + dx, c.y}) || g.isBlocked({c.x, c.y + dy}))) continue;
                const double ng = gScore[i] + (diagonal ? std::numbers::sqrt2 : 1.0);
                if (ng < gScore[idx(n)]) {
                    gScore[idx(n)] = ng;
                    parent[idx(n)] = i;
                    open.push({ng + h(n), idx(n)});
                }
            }
    }
    return std::nullopt;
}

std::vector<Vec3> cellsToWaypoints(const OccupancyGrid& grid, const std::vector<Cell>& path) {
    std::vector<Vec3> wp;
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0 && i + 1 < path.size()) {
            const int d1x = path[i].x - path[i - 1].x, d1y = path[i].y - path[i - 1].y;
            const int d2x = path[i + 1].x - path[i].x, d2y = path[i + 1].y - path[i].y;
            if (d1x == d2x && d1y == d2y) continue;  // straight run, skip middle cell
        }
        wp.push_back(grid.cellCenter(path[i]));
    }
    return wp;
}

}  // namespace r3d
