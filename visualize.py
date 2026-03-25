#!/usr/bin/env python3
"""
visualize.py — File-based visualizer for the SE(2) A* planner.

Reads planner.cfg, query.cfg, se2_waypoints.txt, and controls.txt from disk.
No sockets — run independently after the planner produces its output files.

Usage:
  python3 visualize.py [options]

Options:
  --config <path>      Static config   (default: planner.cfg)
  --query  <path>      Dynamic config   (default: query.cfg)
  --waypoints <path>   Waypoints file   (default: se2_waypoints.txt)
  --controls <path>    Controls file    (default: controls.txt)
  --out <path>         Output image     (default: plan_viz.png)
  --save-only          Don't show interactive window
"""

import math
import sys

import matplotlib
if "--save-only" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as mtransforms
import numpy as np


def parse_cfg(path: str) -> dict:
    """Parse a key=value config file (same format as planner.cfg / query.cfg)."""
    cfg = {}
    obstacles = {}
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or "=" not in line:
                continue
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip()
            if key.startswith("obs."):
                parts = key.split(".")
                if len(parts) == 3:
                    idx = int(parts[1])
                    field = parts[2]
                    if idx not in obstacles:
                        obstacles[idx] = {}
                    obstacles[idx][field] = float(val)
            else:
                try:
                    cfg[key] = float(val)
                except ValueError:
                    cfg[key] = val
    # Convert obstacles dict to sorted list
    if obstacles:
        max_idx = max(obstacles.keys())
        obs_list = []
        for i in range(max_idx + 1):
            if i in obstacles:
                ob = obstacles[i]
                obs_list.append({
                    "x": ob.get("x", 0.0),
                    "y": ob.get("y", 0.0),
                    "theta": ob.get("theta", 0.0),
                    "length": ob.get("length", 0.35),
                    "width": ob.get("width", 0.25),
                })
        cfg["_obstacles"] = obs_list
    else:
        cfg["_obstacles"] = []
    return cfg


def load_waypoints(path: str) -> np.ndarray:
    """Load se2_waypoints.txt: x, y, theta per line."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [float(v) for v in line.replace(",", " ").split()]
            if len(parts) >= 3:
                rows.append(parts[:3])
    return np.array(rows)


def load_controls(path: str) -> list:
    """Load controls.txt: vx, vy, vtheta per line. Duration from config."""
    ctrls = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [float(v) for v in line.replace(",", " ").split()]
            if len(parts) >= 3:
                ctrls.append(parts[:3])
    return ctrls


def draw_panel(ax, title, cfg, obstacles, wp, robot_r, safety):
    """Set up a single panel with map bounds, obstacles, start/goal markers."""
    x_min = cfg["x_min"]
    x_max = cfg["x_max"]
    y_min = cfg["y_min"]
    y_max = cfg["y_max"]

    ax.set_xlim(x_min - 0.15, x_max + 0.15)
    ax.set_ylim(y_min - 0.15, y_max + 0.15)
    ax.set_aspect("equal")
    ax.set_xlabel("x  (m)")
    ax.set_ylabel("y  (m)")
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.axhline(0, color="k", linewidth=0.4)
    ax.axvline(0, color="k", linewidth=0.4)

    # Map boundary
    border = patches.Rectangle(
        (x_min, y_min), x_max - x_min, y_max - y_min,
        linewidth=1.5, edgecolor="gray", facecolor="none",
        linestyle="--", label="Map bounds")
    ax.add_patch(border)

    # Origin
    ax.plot(0, 0, "k+", markersize=12, markeredgewidth=1.5)
    ax.annotate("origin", (0, 0), textcoords="offset points",
                xytext=(6, -12), fontsize=7, color="gray")

    # Obstacles
    for i, ob in enumerate(obstacles):
        cx, cy, ct = ob["x"], ob["y"], ob["theta"]
        ol, ow = ob["length"], ob["width"]
        t = mtransforms.Affine2D().rotate_around(cx, cy, ct) + ax.transData
        r_true = patches.Rectangle(
            (cx - ol / 2, cy - ow / 2), ol, ow, transform=t,
            linewidth=1.2, edgecolor="red", facecolor="salmon",
            alpha=0.5, label=f"Obs {i}")
        ax.add_patch(r_true)
        inf = robot_r + safety
        r_inf = patches.Rectangle(
            (cx - ol / 2 - inf, cy - ow / 2 - inf),
            ol + 2 * inf, ow + 2 * inf, transform=t,
            linewidth=0.8, edgecolor="red", facecolor="none",
            linestyle=":", alpha=0.7)
        ax.add_patch(r_inf)

    # Start marker
    s_al = robot_r * 0.9
    sx, sy, st = wp[0]
    ax.add_patch(plt.Circle((sx, sy), robot_r, fill=False,
                             edgecolor="green", linewidth=1.8, zorder=5))
    ax.annotate("", xy=(sx + s_al * math.cos(st), sy + s_al * math.sin(st)),
                xytext=(sx, sy),
                arrowprops=dict(arrowstyle="-|>", color="green",
                                lw=2.0, mutation_scale=12), zorder=6)
    ax.annotate("START", (sx, sy), textcoords="offset points",
                xytext=(8, 8), fontsize=8, color="green", fontweight="bold")

    # Goal marker
    gx, gy, gt = wp[-1]
    ax.add_patch(plt.Circle((gx, gy), robot_r, fill=False,
                             edgecolor="red", linewidth=1.8, zorder=5))
    ax.annotate("", xy=(gx + s_al * math.cos(gt), gy + s_al * math.sin(gt)),
                xytext=(gx, gy),
                arrowprops=dict(arrowstyle="-|>", color="red",
                                lw=2.0, mutation_scale=12), zorder=6)
    ax.annotate("GOAL", (gx, gy), textcoords="offset points",
                xytext=(8, 8), fontsize=8, color="red", fontweight="bold")


def main():
    config_path = "planner.cfg"
    query_path = "query.cfg"
    waypoints_path = "se2_waypoints.txt"
    controls_path = "controls.txt"
    out_file = "plan_viz.png"
    show = True

    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--config" and i + 1 < len(args):
            config_path = args[i + 1]; i += 2
        elif args[i] == "--query" and i + 1 < len(args):
            query_path = args[i + 1]; i += 2
        elif args[i] == "--waypoints" and i + 1 < len(args):
            waypoints_path = args[i + 1]; i += 2
        elif args[i] == "--controls" and i + 1 < len(args):
            controls_path = args[i + 1]; i += 2
        elif args[i] == "--out" and i + 1 < len(args):
            out_file = args[i + 1]; i += 2
        elif args[i] == "--save-only":
            show = False; i += 1
        else:
            i += 1

    # Load configs (query overlays planner, same as C++ side)
    cfg = parse_cfg(config_path)
    obstacles = cfg.pop("_obstacles", [])
    query = parse_cfg(query_path)
    # Query may also define obstacles (gallery test configs)
    query_obs = query.pop("_obstacles", [])
    if query_obs:
        obstacles = query_obs
    cfg.update(query)

    robot_r = cfg["robot_radius"]
    safety = cfg["safety_margin"]
    arrow_len = robot_r * 0.6
    step_time = cfg.get("step_time", 2.0)

    # Load data files
    wp = load_waypoints(waypoints_path)
    raw_ctrls = load_controls(controls_path)
    # Add duration from config
    ctrls = [(vx, vy, vth, step_time) for vx, vy, vth in raw_ctrls]

    # ── Figure ──
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle("SE(2) A* Planner — Waypoints & Controller",
                 fontsize=14, fontweight="bold")

    draw_panel(ax1, "A* Waypoints", cfg, obstacles, wp, robot_r, safety)
    draw_panel(ax2, "Controller Commands", cfg, obstacles, wp, robot_r, safety)

    # ── Left panel: waypoints ──
    ax1.plot(wp[:, 0], wp[:, 1], "b.-", linewidth=1.2, markersize=6,
             alpha=0.8, label="Path", zorder=3)
    for idx, (x, y, th) in enumerate(wp):
        dx = arrow_len * math.cos(th)
        dy = arrow_len * math.sin(th)
        ax1.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                      arrowprops=dict(arrowstyle="->", color="dodgerblue",
                                      lw=1.4), zorder=5)
        ax1.add_patch(plt.Circle((x, y), robot_r, fill=False,
                                  edgecolor="cornflowerblue",
                                  linewidth=0.6, linestyle="--", alpha=0.35,
                                  zorder=2))
        ax1.annotate(str(idx), (x, y), textcoords="offset points",
                      xytext=(-6, 6), fontsize=6, color="navy", alpha=0.8)
    ax1.legend(loc="upper left", fontsize=8)

    # ── Right panel: controller replay ──
    cx, cy, ct = float(wp[0, 0]), float(wp[0, 1]), float(wp[0, 2])
    traj_x, traj_y = [cx], [cy]
    ctrl_positions = [(cx, cy, ct)]

    for vx, vy, vtheta, dur in ctrls:
        fwd = vx * dur
        world_dx = fwd * math.cos(ct)
        world_dy = fwd * math.sin(ct)
        cx += world_dx
        cy += world_dy
        ct += vtheta * dur
        traj_x.append(cx)
        traj_y.append(cy)
        ctrl_positions.append((cx, cy, ct))

    ax2.plot(traj_x, traj_y, "m.-", linewidth=1.2, markersize=6,
             alpha=0.8, label="Controller traj", zorder=3)

    # Translation arrows
    px, py, pt = float(wp[0, 0]), float(wp[0, 1]), float(wp[0, 2])
    for vx, vy, vtheta, dur in ctrls:
        fwd = vx * dur
        if abs(fwd) > 1e-6:
            world_dx = fwd * math.cos(pt)
            world_dy = fwd * math.sin(pt)
            ax2.annotate("", xy=(px + world_dx, py + world_dy), xytext=(px, py),
                          arrowprops=dict(arrowstyle="-|>", color="purple",
                                          lw=1.0), zorder=4)
            px += world_dx
            py += world_dy
        pt += vtheta * dur

    # Heading arrows + circles
    for idx, (x, y, th) in enumerate(ctrl_positions):
        adx = arrow_len * math.cos(th)
        ady = arrow_len * math.sin(th)
        ax2.annotate("", xy=(x + adx, y + ady), xytext=(x, y),
                      arrowprops=dict(arrowstyle="->", color="orchid",
                                      lw=1.4), zorder=5)
        ax2.add_patch(plt.Circle((x, y), robot_r, fill=False,
                                  edgecolor="plum",
                                  linewidth=0.6, linestyle="--", alpha=0.35,
                                  zorder=2))
        ax2.annotate(str(idx), (x, y), textcoords="offset points",
                      xytext=(-6, 6), fontsize=6, color="purple", alpha=0.7)

    ax2.legend(loc="upper left", fontsize=8)

    fig.text(0.5, 0.01,
             "Blue arrows = heading at waypoint  |  "
             "Purple arrows = forward drive (vx*t)  |  "
             "Dotted red = inflated obstacle  |  Green circle = robot radius",
             ha="center", fontsize=8, color="gray")

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    fig.savefig(out_file, dpi=150, bbox_inches="tight")
    print(f"Saved {out_file}")
    if show:
        plt.show()


if __name__ == "__main__":
    main()
