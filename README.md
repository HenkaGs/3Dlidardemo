# robot3d — 3D LiDAR + A* navigation demo (C++20)

A small robotics demo: a planar mobile robot drives through a 3D world, senses it with a
simulated 3D LiDAR, builds a 2.5D occupancy grid, plans with A*, follows the path with a
waypoint controller, and replans when the LiDAR reveals an obstacle that blocks the route.
No ROS, no physics engine, no SLAM — just the core pipeline, kept small enough to explain.

## 1. What it does

* The world is a ground plane with box obstacles (walls, boxes, a rock) and a goal.
* The robot starts with a **prior map** that contains only the *known* obstacles (grey).
  Two *unknown* obstacles (orange) are only discovered when the LiDAR hits them.
* Each tick the robot scans, updates the grid, replans if its route is now blocked, and drives.
* The viewer (raylib) shows the point cloud (red = obstacle hits, green = ground hits), the
  occupancy grid overlay, the planned path (green) and the robot.

## 2. Architecture

```text
3D Environment
      ↓
   3D LiDAR            (ray casts; LiDAR → Robot → World transforms)
      ↓
   Point Cloud
      ↓
Obstacle Detection     (drop ground returns, keep points in a height band)
      ↓
2.5D Occupancy Grid    (flatten to X/Y, inflate by robot radius)
      ↓
      A*               (8-connected, Euclidean heuristic)
      ↓
   Waypoints
      ↓
  Controller           (rotate toward waypoint, drive forward)
      ↓
    Robot
```

| Module | Role |
|---|---|
| `Geometry` | `Vec3`, `Mat3`, `Transform`, `Pose`, ray/AABB and ray/ground intersection |
| `Environment` | ground + box obstacles, closest-hit ray casting, demo scene |
| `Lidar` | ray pattern, scan, point-cloud → obstacle points |
| `AStar` | `OccupancyGrid` (with inflation) and the A* planner |
| `Controller` | waypoint follower |
| `Robot` | planar kinematics (`drive(v, ω, dt)`) |
| `Simulation` | glues the pipeline together, **no rendering** |
| `main.cpp` | raylib viewer only |

Everything except `main.cpp` builds into `robot3d_core` with no graphics dependency, so it
can be tested and run headless.

## 3. How the LiDAR works

For each vertical angle (default −20°, −10°, 0°, +10°) and each of 72 azimuths, the sensor
builds a unit direction in the **LiDAR frame**

    d = (cos(el)·cos(az), cos(el)·sin(az), sin(el))

rotates it into the world, and casts a ray from the sensor origin (mounted 0.5 m above the
robot origin). `Environment::raycast` returns the **closest** hit among all boxes and the
ground plane within the 8 m max range. A ray/box test uses the *slab method*: intersect the
ray with the three pairs of axis-aligned planes, keep the overlap of the three `[t_near, t_far]`
intervals, and the entry distance `t_near` is the hit if the overlap is non-empty.
Each return stores its point in the LiDAR frame (`range · d`) — exactly what a real sensor
reports. Downward rays hit the ground (green); those are filtered out before mapping.

## 4. Coordinate transformations

A rigid transform from a child frame to its parent is

    p_parent = R · p_child + t          (local → world)
    p_child  = Rᵀ · (p_parent − t)      (world → local, since R⁻¹ = Rᵀ)

`R` is built from Euler angles as `R = Rz(yaw)·Ry(pitch)·Rx(roll)`. Transforms compose by
matrix product: `T_a←c = T_a←b · T_b←c`, i.e. `(R_ab·R_bc, R_ab·t_bc + t_ab)`. The LiDAR uses

    T_world←lidar = T_world←robot · T_robot←lidar

so a point 2 m ahead of the sensor on a robot at (2, 1) facing +Y ends up at (2, 3, 0.5).
See `Transform::toParent / toLocal / operator* / inverse` and `localToWorld / worldToLocal`.

## 5. Occupancy grid

The workspace (12 m × 12 m) is a 48 × 48 grid of 0.25 m cells: `0 = free`, `1 = obstacle`.
It starts from the prior map (known boxes are rasterized by footprint). Each scan, every
obstacle hit point (between 5 cm and 2 m high) is nudged 5 cm along its ray — so it lands
*inside* the obstacle rather than on its face — and its cell is marked occupied.

**Inflation:** every occupied cell also blocks all cells within `robot radius + margin`
(0.35 + 0.10 m → 2 cells). The planner can then treat the robot as a single point.
Cells outside the grid count as blocked. Unknown space is assumed free.

## 6. A*

A* searches cells from the start; each cell has `g` (cost so far) and `f = g + h`, and the
open cell with the lowest `f` is expanded first. Moves are 8-connected with cost 1
(orthogonal) or √2 (diagonal); a diagonal move is rejected if either orthogonal neighbour is
blocked (no corner cutting). The heuristic `h` is the Euclidean distance to the goal, which
never overestimates the true cost, so the first time the goal is popped the path is optimal on
the grid. If the open set empties, there is no path (`std::nullopt`). Straight runs are merged
into single waypoints.

## 7. Why 2.5D?

The LiDAR and world are genuinely 3D, but a ground robot only needs to know *where it can
drive*, not the full volume. Any obstacle that intersects the robot's height band is simply
"blocked" in the X/Y plane, so a 2D grid captures everything the planner needs at a tiny
fraction of the memory and search cost of a 3D voxel map (A* over a 48×48 grid vs. 48×48×N
voxels, with a much larger branching factor). The third dimension still matters for
*perception* — it is used to reject ground returns and to filter by height — and is simply
collapsed afterwards. The trade-off: no overhangs, ramps, or multi-level terrain.

## 8. Build and run

Requirements: a C++20 compiler and CMake ≥ 3.16. raylib 5.0 and GoogleTest 1.14 are used from
the system if found, otherwise downloaded by CMake (needs internet on first configure).
On Linux, raylib needs the usual X11/OpenGL dev packages
(e.g. `sudo apt install libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev libgl1-mesa-dev`).

```bash
cmake -S . -B build
cmake --build build -j
./build/robot3d              # interactive viewer
./build/robot3d --headless   # run the simulation without a window, print the result
ctest --test-dir build --output-on-failure
```

Viewer controls: drag = orbit, wheel = zoom, `SPACE` pause, `R` reset, `L` show rays,
`G` toggle grid, `↑/↓` sim speed. Options: `-DROBOT3D_BUILD_VIZ=OFF` (core + tests only),
`-DROBOT3D_BUILD_TESTS=OFF`.

## 9. Limitations and ideas

* Perfect localization — the robot knows its true pose (no SLAM / odometry noise).
* Noise-free, instantaneous LiDAR; obstacles are axis-aligned boxes only.
* Obstacles are only ever *added* to the grid, never cleared, so moving obstacles are not handled.
* Only the surface the LiDAR sees is marked, so the robot replans several times while
  rounding a large unknown box.
* Replanning is triggered only when newly seen cells block the remaining route; there is no
  recovery behaviour if the robot ends up boxed in (it reports `NO PATH`).
* Kinematic robot: it can turn in place and has no acceleration limits.

Possible improvements: probabilistic (log-odds) grid with free-space ray clearing, noise
models, D* Lite / incremental replanning, path smoothing, a pure-pursuit controller, and
height-aware traversability (slopes, steps) for a true 2.5D elevation map.
