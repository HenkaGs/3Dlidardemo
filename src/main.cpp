// Visualization only: all robotics logic lives in robot3d_core (Simulation & friends).
// raylib is Y-up; the simulation is Z-up, so (x, y, z) -> (x, z, -y).
#include <cstdio>
#include <cstring>

#include "Simulation.hpp"
#include "raylib.h"

namespace {

Vector3 rl(const r3d::Vec3& v) { return {static_cast<float>(v.x), static_cast<float>(v.z), static_cast<float>(-v.y)}; }

void drawWorld(const r3d::Simulation& sim, bool showRays, bool showGrid) {
    const auto& env = sim.environment();
    const auto& grid = sim.grid();

    // Ground plane + grid lines
    DrawPlane({0, 0, 0}, {static_cast<float>(env.xMax - env.xMin), static_cast<float>(env.yMax - env.yMin)},
              Color{70, 80, 70, 255});
    DrawGrid(12, 1.0f);

    // Obstacles: grey = in prior map, orange = unknown until the LiDAR sees them
    for (const auto& ob : env.obstacles) {
        const auto c = ob.box.center();
        const auto s = ob.box.size();
        const Vector3 pos{static_cast<float>(c.x), static_cast<float>(s.z / 2), static_cast<float>(-c.y)};
        const Vector3 size{static_cast<float>(s.x), static_cast<float>(s.z), static_cast<float>(s.y)};
        DrawCubeV(pos, size, ob.known ? Color{150, 150, 160, 255} : Color{230, 140, 40, 255});
        DrawCubeWiresV(pos, size, Color{30, 30, 30, 255});
    }

    // Occupancy grid overlay: red = obstacle cell, translucent yellow = inflated margin
    if (showGrid) {
        const float r = static_cast<float>(grid.resolution());
        for (int y = 0; y < grid.height(); ++y)
            for (int x = 0; x < grid.width(); ++x) {
                const r3d::Cell c{x, y};
                if (!grid.isBlocked(c)) continue;
                Vector3 p = rl(grid.cellCenter(c));
                p.y = 0.02f;
                DrawCubeV(p, {r * 0.95f, 0.02f, r * 0.95f},
                          grid.isOccupied(c) ? Color{220, 40, 40, 200} : Color{240, 200, 40, 90});
            }
    }

    // Planned path (remaining waypoints)
    const auto& wp = sim.controller().waypoints();
    Vector3 prev = rl(sim.robot().pose().position + r3d::Vec3{0, 0, 0.1});
    for (size_t i = sim.controller().nextIndex(); i < wp.size(); ++i) {
        const Vector3 p = rl(wp[i] + r3d::Vec3{0, 0, 0.1});
        DrawLine3D(prev, p, GREEN);
        DrawSphere(p, 0.06f, LIME);
        prev = p;
    }

    // LiDAR point cloud (+ optional rays)
    const auto& scan = sim.lastScan();
    const Vector3 o = rl(scan.originWorld());
    for (const auto& r : scan.returns) {
        if (r.type == r3d::HitType::None) continue;
        const Vector3 p = rl(scan.pointWorld(r));
        if (showRays) DrawLine3D(o, p, Fade(SKYBLUE, 0.15f));
        DrawSphere(p, 0.035f, r.type == r3d::HitType::Obstacle ? RED : Color{80, 200, 120, 255});
    }

    // Goal and robot
    const Vector3 g = rl(env.goal + r3d::Vec3{0, 0, 0.3});
    DrawSphere(g, 0.3f, GOLD);
    const auto& pose = sim.robot().pose();
    DrawCylinder(rl(pose.position), 0.3f, 0.3f, 0.4f, 20, BLUE);
    DrawSphere(rl(pose.position + r3d::Vec3{0, 0, 0.5}), 0.1f, DARKBLUE);  // LiDAR dome
    const r3d::Vec3 nose{std::cos(sim.robot().yaw()) * 0.3, std::sin(sim.robot().yaw()) * 0.3, 0.3};
    DrawSphere(rl(pose.position + nose), 0.08f, WHITE);                    // heading marker
}

int runHeadless() {
    r3d::Simulation sim;
    while (sim.status() == r3d::SimStatus::Navigating && sim.elapsed() < 180.0) sim.step(1.0 / 30.0);
    const auto& p = sim.robot().pose().position;
    std::printf("status=%s time=%.1fs replans=%d final=(%.2f, %.2f)\n",
                sim.status() == r3d::SimStatus::GoalReached ? "goal reached" : "not reached", sim.elapsed(),
                sim.replanCount(), p.x, p.y);
    return sim.status() == r3d::SimStatus::GoalReached ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--headless") == 0) return runHeadless();

    InitWindow(1280, 720, "robot3d - LiDAR + A* navigation");
    SetTargetFPS(60);

    r3d::Simulation sim;
    bool paused = false, showRays = false, showGrid = true;
    float camYaw = 0.6f, camPitch = 0.9f, camDist = 20.0f, speed = 1.0f;
    const double simDt = 1.0 / 60.0;
    double acc = 0.0;

    while (!WindowShouldClose()) {
        // Input
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_R)) { sim.reset(); acc = 0; }
        if (IsKeyPressed(KEY_L)) showRays = !showRays;
        if (IsKeyPressed(KEY_G)) showGrid = !showGrid;
        if (IsKeyPressed(KEY_UP)) speed = speed * 2.0f > 8.0f ? 8.0f : speed * 2.0f;
        if (IsKeyPressed(KEY_DOWN)) speed = speed * 0.5f < 0.25f ? 0.25f : speed * 0.5f;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            const Vector2 d = GetMouseDelta();
            camYaw -= d.x * 0.005f;
            camPitch += d.y * 0.005f;
            if (camPitch < 0.15f) camPitch = 0.15f;
            if (camPitch > 1.5f) camPitch = 1.5f;
        }
        camDist -= GetMouseWheelMove() * 1.0f;
        if (camDist < 6.0f) camDist = 6.0f;
        if (camDist > 40.0f) camDist = 40.0f;

        // Fixed-step simulation, independent of render frame rate
        if (!paused) {
            acc += GetFrameTime() * speed;
            while (acc >= simDt) { sim.step(simDt); acc -= simDt; }
        }

        Camera3D cam{};
        cam.target = {0, 0, 0};
        cam.position = {camDist * std::cos(camPitch) * std::sin(camYaw), camDist * std::sin(camPitch),
                        camDist * std::cos(camPitch) * std::cos(camYaw)};
        cam.up = {0, 1, 0};
        cam.fovy = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;

        BeginDrawing();
        ClearBackground(Color{25, 28, 35, 255});
        BeginMode3D(cam);
        drawWorld(sim, showRays, showGrid);
        EndMode3D();

        const char* st = sim.status() == r3d::SimStatus::GoalReached ? "GOAL REACHED"
                         : sim.status() == r3d::SimStatus::NoPath    ? "NO PATH"
                                                                      : "navigating";
        DrawText(TextFormat("%s   t=%.1fs   replans=%d   speed x%.2f%s", st, sim.elapsed(), sim.replanCount(),
                            speed, paused ? "   [paused]" : ""), 12, 10, 20, RAYWHITE);
        DrawText("drag: orbit | wheel: zoom | SPACE: pause | R: reset | L: rays | G: grid | UP/DOWN: speed",
                 12, 36, 16, LIGHTGRAY);
        DrawText("grey = known obstacle   orange = unknown (found by LiDAR)   red = LiDAR hit   green = path",
                 12, 58, 16, LIGHTGRAY);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
