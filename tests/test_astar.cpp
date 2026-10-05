#include <gtest/gtest.h>

#include <cmath>

#include "AStar.hpp"

using namespace r3d;

namespace {
OccupancyGrid makeGrid(double inflation = 0.0) { return OccupancyGrid(0, 0, 20, 20, 1.0, inflation); }
}

TEST(AStar, OpenGridDiagonalIsShortest) {
    auto g = makeGrid();
    auto path = planPath(g, {0, 0}, {10, 10});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (Cell{0, 0}));
    EXPECT_EQ(path->back(), (Cell{10, 10}));
    EXPECT_EQ(path->size(), 11u);  // 10 diagonal steps
}

TEST(AStar, PathAvoidsObstaclesAndGoesAround) {
    auto g = makeGrid();
    for (int y = 0; y < 15; ++y) g.setOccupied({10, y});  // wall with a gap at y >= 15
    auto path = planPath(g, {2, 2}, {18, 2});
    ASSERT_TRUE(path.has_value());
    for (const Cell& c : *path) EXPECT_FALSE(g.isBlocked(c));
    EXPECT_GT(path->size(), 16u);  // must detour around the wall
}

TEST(AStar, ImpossiblePathReturnsNullopt) {
    auto g = makeGrid();
    for (int y = 0; y < 20; ++y) g.setOccupied({10, y});  // full wall
    EXPECT_FALSE(planPath(g, {2, 2}, {18, 2}).has_value());
}

TEST(AStar, BlockedOrOutOfBoundsGoalFails) {
    auto g = makeGrid();
    g.setOccupied({5, 5});
    EXPECT_FALSE(planPath(g, {0, 0}, {5, 5}).has_value());
    EXPECT_FALSE(planPath(g, {0, 0}, {50, 5}).has_value());
}

TEST(AStar, NoDiagonalCornerCutting) {
    auto g = makeGrid();
    g.setOccupied({1, 0});
    g.setOccupied({0, 1});  // (0,0)->(1,1) would squeeze between two obstacles
    EXPECT_FALSE(planPath(g, {0, 0}, {5, 5}).has_value());
}

TEST(OccupancyGrid, InflationBlocksNeighbours) {
    auto g = makeGrid(2.0);
    g.setOccupied({10, 10});
    EXPECT_TRUE(g.isOccupied({10, 10}));
    EXPECT_FALSE(g.isOccupied({11, 10}));
    EXPECT_TRUE(g.isBlocked({12, 10}));
    EXPECT_FALSE(g.isBlocked({13, 10}));
}

TEST(AStar, WaypointsMergeStraightRuns) {
    auto g = makeGrid();
    auto path = planPath(g, {0, 0}, {10, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(cellsToWaypoints(g, *path).size(), 2u);  // only start and end remain
}
