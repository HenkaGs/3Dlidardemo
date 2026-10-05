// Integration test: full pipeline, headless.
#include <gtest/gtest.h>

#include <algorithm>

#include "Simulation.hpp"

using namespace r3d;

TEST(Navigation, ReachesGoalWithReplanningAndNoCollisions) {
    Simulation sim;
    const double dt = 1.0 / 30.0;
    double minClearance = 1e9;
    for (int i = 0; i < 30 * 120 && sim.status() == SimStatus::Navigating; ++i) {
        sim.step(dt);
        const Vec3 p = sim.robot().pose().position;
        for (const auto& ob : sim.environment().obstacles) {
            const double dx = std::max({ob.box.min.x - p.x, 0.0, p.x - ob.box.max.x});
            const double dy = std::max({ob.box.min.y - p.y, 0.0, p.y - ob.box.max.y});
            minClearance = std::min(minClearance, std::hypot(dx, dy));
        }
    }
    EXPECT_EQ(sim.status(), SimStatus::GoalReached);
    EXPECT_GE(sim.replanCount(), 1);                 // unknown obstacles forced a replan
    EXPECT_GT(minClearance, sim.robot().radius());   // robot body never touched an obstacle
    const Vec3 p = sim.robot().pose().position;
    EXPECT_LT((p - sim.environment().goal).normXY(), 0.2);
}
