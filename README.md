# SE(2) A* Path Planner

A discrete grid-based A* planner for SE(2) (x, y, θ) with unicycle controller output.

![Plan Visualization](plan_viz.png)

## Structure

```
├── planner.cc            # A* planner + controller (single source file)
├── visualize.py          # Matplotlib two-panel visualization (file-based, no sockets)
├── Makefile              # Builds astar binary
├── planner.cfg           # Static config (map, grid, robot, obstacle dimensions)
├── query.cfg             # Dynamic config (start/goal poses, obstacle poses)
├── test_gallery.sh       # Gallery test runner (10 scenarios)
├── gallery/              # Test scenarios + reference images
├── se2_waypoints.txt     # Generated waypoints (x, y, θ)
├── controls.txt          # Generated controls (vx, vy, vθ) — vy always 0
└── plan_viz.png          # Generated visualization
```

## Build

```bash
make          # builds astar binary
make clean    # removes all build artifacts
```

Requires: `g++` with C++17 support.

## Usage

The planner binary supports two CLI phases that can be run independently:

### 1. Plan a path

```bash
./astar --plan                                      # writes se2_waypoints.txt
./astar --plan --config planner.cfg --query query.cfg  # explicit config paths
```

Outputs `se2_waypoints.txt` with one `x, y, θ` waypoint per line.

### 2. Generate controls

```bash
./astar --gen_control                               # reads se2_waypoints.txt, writes controls.txt
```

Outputs comma-separated `vx, vy, vθ` per line (unicycle model: `vy` is always `0`).

### Run both phases at once

```bash
./astar                                             # plan + gen_control in one shot
```

### 3. Visualize

```bash
python3 visualize.py                                # reads cfg files + output files, shows plot
python3 visualize.py --save-only                    # save plan_viz.png without displaying
```

The visualizer runs asynchronously — no sockets, just reads from disk. Run it anytime after the planner has produced output files.

```bash
make run      # build + run planner + visualize
make viz      # visualize from existing output files
```

## Configuration

Two config files, both `key = value` format:

**planner.cfg** — static environment and robot settings:

| Key | Description |
|-----|-------------|
| `x_min`, `x_max`, `y_min`, `y_max` | Map bounds (metres) |
| `dx`, `dy`, `n_theta` | Grid resolution |
| `robot_radius` | Robot collision radius |
| `safety_margin` | Extra inflation beyond robot radius |
| `goal_tol` | Euclidean goal tolerance |
| `step_time` | Controller time per step (seconds) |
| `obs.N.length`, `obs.N.width` | Obstacle N dimensions |

**query.cfg** — dynamic per-run settings:

| Key | Description |
|-----|-------------|
| `start_x/y/theta` | Start pose |
| `goal_x/y/theta` | Goal pose |
| `obs.N.x/y/theta` | Obstacle N pose (position + orientation) |

## Testing

```bash
make test       # run 10 gallery scenarios
make sanitize   # run under AddressSanitizer + UBSan + LeakSanitizer
```

## Motion Model

The robot is a **unicycle** — it can only drive forward along its heading (`vx`) and rotate in place (`vθ`). Lateral velocity `vy` is constrained to zero. The planner uses 8 heading bins and produces grid-aligned paths that respect inflated obstacle boundaries.
