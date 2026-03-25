# SE(2) A* Path Planner

Discrete grid-based A* planner over SE(2) (x, y, theta) with velocity controller output. Controls are written line-by-line to `controls.txt`.

![Plan Visualization](plan_viz.png)

## Prerequisites

A C++17 compiler and Python 3 with `matplotlib` and `numpy` (visualization only).

### Linux

```bash
sudo apt install g++ python3 python3-matplotlib python3-numpy   # Debian/Ubuntu
sudo dnf install gcc-c++ python3 python3-matplotlib python3-numpy  # Fedora
```

### macOS

```bash
xcode-select --install          # provides clang++ with C++17 support
pip3 install matplotlib numpy   # or: brew install python-matplotlib
```

To use Homebrew GCC instead of Apple Clang:

```bash
brew install gcc
make CXX=g++-14
```

### Windows

The planner uses POSIX sockets and must be built inside **WSL** (Windows Subsystem for Linux). From a WSL terminal, follow the Linux instructions above.

## Quick Start

```bash
# Edit query.cfg with your start, goal, and obstacles, then:
make run
```

Compiles `planner.cc`, launches `visualize.py` in background, runs the planner which streams results to the visualizer over TCP and writes controls to `controls.txt`.

## Files

```
planner.cc               A* planner + controller + socket viz
planner.cfg              Static config (map, grid, robot, tolerances)
query.cfg                Dynamic config (start, goal, obstacles)
visualize.py             Socket-based matplotlib visualization
Makefile                 Build, run, test, sanitize
test_gallery.sh          Runs all 10 gallery scenarios
gallery/*.cfg            Per-scenario query configs
gallery/planner.cfg      Static config for gallery (larger map)
```

Generated at runtime (gitignored):

```
controls.txt             Output: one line per step (vx, vy, vtheta)
astar                    Compiled binary
plan_viz.png             Visualization image
```

## Config

### `planner.cfg` — Static

| Key | Description |
| --- | --- |
| `x_min/max`, `y_min/max` | Map bounds (metres) |
| `dx`, `dy`, `n_theta` | Grid resolution |
| `robot_radius` | Collision radius |
| `safety_margin` | Extra inflation |
| `goal_tol` | Goal tolerance (Euclidean, metres) |
| `step_time` | Seconds per control step |
| `viz_port` | TCP port for visualizer |
| `delta_x/y/theta` | Post-planning offset |

### `query.cfg` — Dynamic (edit per run)

| Key | Description |
| --- | --- |
| `start_x/y/theta` | Start pose |
| `goal_x/y/theta` | Goal pose |
| `obs.N.x/y/theta/length/width` | Obstacle N (oriented rectangle) |

## Controller Output

Each path segment produces a velocity command written to `controls.txt`, one line per step:

```
vx, vy, vtheta
```

| Field | Description |
| --- | --- |
| `vx` | Forward velocity (m/s) along heading |
| `vy` | Lateral velocity (always 0, unicycle) |
| `vtheta` | Angular velocity (rad/s) |

Each command is held for `step_time` seconds (from `planner.cfg`). The A* planner uses a unicycle model (forward motion only), so the path may arrive at the goal position with a different heading. A final in-place rotation command (`vx=0`) is appended when needed to match `goal_theta`.

## Usage

```bash
make                # compile only
make run            # compile + visualize + plan
make test           # run all 10 gallery test scenarios
make sanitize       # build with ASan/UBSan/LeakSan and run tests
make clean          # remove binary + outputs
./astar --no-viz    # plan without visualization
./astar --output <path>  # write controls to custom path (default: controls.txt)
./astar --config <path> --query <path>
```

## Testing

```bash
make test       # runs test_gallery.sh — 10 scenarios, expects all to find a path
make sanitize   # same tests under AddressSanitizer + UBSan + LeakSanitizer
```

Gallery test configs live in `gallery/*.cfg`. Each matches one of the 10 gallery images below. The gallery uses its own `gallery/planner.cfg` with a larger map.

## Gallery

Tested across 10 scenarios with 0–5 obstacles, varying start/goal headings and layouts.

| | |
| --- | --- |
| ![01](gallery/01_no_obstacles.png) | ![02](gallery/02_single_obstacle.png) |
| No obstacles | Single obstacle |
| ![03](gallery/03_two_obs_corridor.png) | ![04](gallery/04_three_obs_slalom.png) |
| Two obstacles — corridor | Three obstacles — slalom |
| ![05](gallery/05_four_obs_dense.png) | ![06](gallery/06_goal_behind_obs.png) |
| Four obstacles — dense | Goal behind obstacle |
| ![07](gallery/07_diagonal_traverse.png) | ![08](gallery/08_u_turn.png) |
| Diagonal traverse | U-turn |
| ![09](gallery/09_five_obs_course.png) | ![10](gallery/10_lateral_dodge.png) |
| Five obstacles — course | Lateral dodge |
