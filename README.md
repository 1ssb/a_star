# SE(2) A* Path Planner

A discrete grid-based A* planner for SE(2) (x, y, θ) with unicycle controller output.

![Plan Visualization](plan_viz.png)

## Structure

```
├── Makefile              # Top-level — delegates to planner/
├── planner/
│   ├── Makefile          # Builds all C++ targets
│   ├── plan2exec.cc      # A* planner on (x, y, θ) grid
│   ├── waypoint2ctrl.cc  # Converts waypoints → unicycle controls (vx, 0, vθ)
│   ├── ctrl_runner.cc    # Reads controls.txt and calls SetMotion() per step
│   ├── planner_config.h  # Shared types, geometry helpers, config loader
│   └── planner.cfg       # Planner parameters (obstacles, grid, robot)
├── planner.cfg           # Config copy for running from project root
├── visualize.py          # Matplotlib two-panel visualization
├── se2_path_floor.txt    # Generated waypoints (x y θ)
├── controls.txt          # Generated controls (vx, vy, vθ) — vy always 0
└── plan_viz.png          # Generated visualization
```

## Build

```bash
make          # builds plan2exec, waypoint2ctrl, ctrl_runner in planner/
make clean    # removes all build artifacts
```

Requires: `g++` with C++17 support.

## Usage

### 1. Plan a path

```bash
./planner/plan2exec                             # uses planner.cfg in cwd
./planner/plan2exec --config path/to/custom.cfg # custom config
```

Outputs `se2_path_floor.txt` with one `x y θ` waypoint per line.

### 2. Generate controls

```bash
./planner/waypoint2ctrl se2_path_floor.txt -o controls.txt
```

Outputs comma-separated `vx, vy, vθ` per line (unicycle model: `vy` is always `0`).

### 3. Execute controls

```bash
./planner/ctrl_runner controls.txt
```

Calls `SetMotion(vx, vy, vθ)` for each line with a configurable sleep (`TIME_SLEEP` macro, default 5s). Replace the `SetMotion()` stub with your robot API.

### 4. Visualize

```bash
python3 visualize.py
```

Produces `plan_viz.png` — two panels showing the A* waypoints and the controller trajectory with heading arrows, robot radius circles, and inflated obstacle boundaries.

## Configuration

Edit `planner.cfg` to set:

| Key | Description |
|-----|-------------|
| `x_min`, `x_max`, `y_min`, `y_max` | Map bounds (metres) |
| `dx`, `dy`, `n_theta` | Grid resolution |
| `robot_radius` | Robot collision radius |
| `safety_margin` | Extra inflation beyond robot radius |
| `goal_tol` | Euclidean goal tolerance |
| `start_x/y/theta` | Start pose |
| `goal_x/y/theta` | Goal pose |
| `obs.N.x/y/theta/length/width` | Obstacle N (oriented rectangle) |

## Motion Model

The robot is a **unicycle** — it can only drive forward along its heading (`vx`) and rotate in place (`vθ`). Lateral velocity `vy` is constrained to zero. The planner uses 8 heading bins and produces grid-aligned paths that respect inflated obstacle boundaries.
