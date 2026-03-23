#!/usr/bin/env python3
"""
visualize.py — Plot A* waypoints, controller commands, and obstacles.

Reads:
  - se2_path_floor.txt   (x y theta  waypoints)
  - controls.txt         (dx dy delta_theta  per segment)
  - planner.cfg          (obstacles, map bounds, robot radius, start/goal)

Produces a matplotlib figure saved to  plan_viz.png  and displayed.
"""

import math
import re
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch

# ── Config parser ───────────────────────────────────────────
def load_config(path="planner.cfg"):
    cfg = {}
    with open(path) as f:
        for line in f:
            line = line.split("#")[0].strip()
            if "=" not in line:
                continue
            k, v = line.split("=", 1)
            cfg[k.strip()] = v.strip()
    return cfg

def parse_obstacles(cfg):
    obs = {}
    pattern = re.compile(r"^obs\.(\d+)\.(\w+)$")
    for k, v in cfg.items():
        m = pattern.match(k)
        if m:
            idx, field = int(m.group(1)), m.group(2)
            obs.setdefault(idx, {})
            obs[idx][field] = float(v)
    result = []
    for i in sorted(obs):
        o = obs[i]
        result.append(dict(
            x=o.get("x", 0), y=o.get("y", 0), theta=o.get("theta", 0),
            length=o.get("length", 0.3), width=o.get("width", 0.3),
        ))
    return result

# ── Waypoint loader ─────────────────────────────────────────
def load_waypoints(path="se2_path_floor.txt"):
    pts = []
    with open(path) as f:
        next(f)  # skip header
        for line in f:
            parts = line.split()
            if len(parts) >= 3:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(pts)

# ── Controls loader ─────────────────────────────────────────
def load_controls(path="controls.txt"):
    ctrls = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.replace(',', ' ').split()]
            if len(parts) >= 3:
                ctrls.append([float(parts[0]), float(parts[2])])  # vx, vtheta (vy=0 skipped)
    return ctrls

# ── Main ────────────────────────────────────────────────────
def main():
    cfg_path = "planner.cfg"
    wp_path  = "se2_path_floor.txt"
    ctrl_path = "controls.txt"
    if len(sys.argv) > 1: wp_path = sys.argv[1]
    if len(sys.argv) > 2: ctrl_path = sys.argv[2]
    if len(sys.argv) > 3: cfg_path = sys.argv[3]

    cfg = load_config(cfg_path)
    obstacles = parse_obstacles(cfg)
    wp = load_waypoints(wp_path)
    ctrls = load_controls(ctrl_path)

    x_min = float(cfg.get("x_min", -1))
    x_max = float(cfg.get("x_max", 1.5))
    y_min = float(cfg.get("y_min", -1))
    y_max = float(cfg.get("y_max", 2.75))
    robot_r = float(cfg.get("robot_radius", 0.25))
    safety  = float(cfg.get("safety_margin", 0.02))

    start_x = float(cfg.get("start_x", 0))
    start_y = float(cfg.get("start_y", 0))
    start_t = float(cfg.get("start_theta", 0))
    goal_x  = float(cfg.get("goal_x", 0))
    goal_y  = float(cfg.get("goal_y", 0))
    goal_t  = float(cfg.get("goal_theta", 0))

    # ── Figure ──────────────────────────────────────────────
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle("SE(2) A* Planner — Waypoints & Controller", fontsize=14,
                 fontweight="bold")

    for ax, title in [(ax1, "A* Waypoints"), (ax2, "Controller Commands")]:
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

        # Origin marker
        ax.plot(0, 0, "k+", markersize=12, markeredgewidth=1.5)
        ax.annotate("origin", (0, 0), textcoords="offset points",
                     xytext=(6, -12), fontsize=7, color="gray")

        # Obstacles (true + inflated)
        for i, ob in enumerate(obstacles):
            cx, cy, ct = ob["x"], ob["y"], ob["theta"]
            ol, ow = ob["length"], ob["width"]
            # True obstacle
            rect = patches.Rectangle(
                (-ol / 2, -ow / 2), ol, ow,
                angle=math.degrees(ct), rotation_point="center",
                linewidth=1.2, edgecolor="red", facecolor="salmon",
                alpha=0.6, label=f"Obs {i}" if i < 2 else None)
            rect.set_xy((cx - ol / 2 * math.cos(ct) + ow / 2 * math.sin(ct),
                         cy - ol / 2 * math.sin(ct) - ow / 2 * math.cos(ct)))
            # Use a transform for proper rotation about center
            t = (plt.matplotlib.transforms.Affine2D()
                 .rotate_around(cx, cy, ct) + ax.transData)
            r_true = patches.Rectangle(
                (cx - ol / 2, cy - ow / 2), ol, ow, transform=t,
                linewidth=1.2, edgecolor="red", facecolor="salmon",
                alpha=0.5, label=f"Obs {i}")
            ax.add_patch(r_true)
            # Inflated
            inf = robot_r + safety
            r_inf = patches.Rectangle(
                (cx - ol / 2 - inf, cy - ow / 2 - inf),
                ol + 2 * inf, ow + 2 * inf, transform=t,
                linewidth=0.8, edgecolor="red", facecolor="none",
                linestyle=":", alpha=0.7)
            ax.add_patch(r_inf)

        # Start marker — use actual first waypoint pose
        sx, sy, st = float(wp[0, 0]), float(wp[0, 1]), float(wp[0, 2])
        start_circ = plt.Circle((sx, sy), robot_r,
                                 fill=False, edgecolor="green", linewidth=1.8,
                                 linestyle="-", zorder=5)
        ax.add_patch(start_circ)
        s_al = robot_r * 0.9
        ax.annotate("", xy=(sx + s_al * math.cos(st),
                             sy + s_al * math.sin(st)),
                     xytext=(sx, sy),
                     arrowprops=dict(arrowstyle="-|>", color="green",
                                     lw=2.0, mutation_scale=12), zorder=6)
        ax.annotate("START", (sx, sy), textcoords="offset points",
                     xytext=(8, 8), fontsize=8, color="green", fontweight="bold")

        # Goal marker — use actual last waypoint pose
        gx, gy, gt = float(wp[-1, 0]), float(wp[-1, 1]), float(wp[-1, 2])
        goal_circ = plt.Circle((gx, gy), robot_r,
                                fill=False, edgecolor="red", linewidth=1.8,
                                linestyle="-", zorder=5)
        ax.add_patch(goal_circ)
        ax.annotate("", xy=(gx + s_al * math.cos(gt),
                             gy + s_al * math.sin(gt)),
                     xytext=(gx, gy),
                     arrowprops=dict(arrowstyle="-|>", color="red",
                                     lw=2.0, mutation_scale=12), zorder=6)
        ax.annotate("GOAL", (gx, gy), textcoords="offset points",
                     xytext=(8, 8), fontsize=8, color="red", fontweight="bold")

    # ── Left panel: waypoints with heading arrows + radius circles ──
    ax1.plot(wp[:, 0], wp[:, 1], "b.-", linewidth=1.2, markersize=6,
             alpha=0.8, label="Path", zorder=3)
    arrow_len = robot_r * 0.6
    for i, (x, y, th) in enumerate(wp):
        dx = arrow_len * math.cos(th)
        dy = arrow_len * math.sin(th)
        ax1.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                      arrowprops=dict(arrowstyle="->", color="dodgerblue",
                                      lw=1.4), zorder=5)
        ax1.add_patch(plt.Circle((x, y), robot_r,
                                  fill=False, edgecolor="cornflowerblue",
                                  linewidth=0.6, linestyle="--", alpha=0.35,
                                  zorder=2))
        ax1.annotate(str(i), (x, y), textcoords="offset points",
                      xytext=(-6, 6), fontsize=6, color="navy", alpha=0.8)
    ax1.legend(loc="upper left", fontsize=8)

    # ── Right panel: controller commands (unicycle: vx, vtheta) ──
    # Replay controls: vx drives along current heading, then rotate by vtheta
    cx, cy, ct = wp[0]
    traj_x, traj_y = [cx], [cy]
    ctrl_positions = [(cx, cy, ct)]

    for vx, vtheta in ctrls:
        # translate forward along current heading
        world_dx = vx * math.cos(ct)
        world_dy = vx * math.sin(ct)
        cx += world_dx
        cy += world_dy
        ct += vtheta
        traj_x.append(cx)
        traj_y.append(cy)
        ctrl_positions.append((cx, cy, ct))

    ax2.plot(traj_x, traj_y, "m.-", linewidth=1.2, markersize=6,
             alpha=0.8, label="Controller traj", zorder=3)

    # Translation arrows (forward drive along heading)
    px, py, pt = wp[0]
    for idx, (vx, vtheta) in enumerate(ctrls):
        if abs(vx) > 1e-6:
            world_dx = vx * math.cos(pt)
            world_dy = vx * math.sin(pt)
            ax2.annotate("", xy=(px + world_dx, py + world_dy), xytext=(px, py),
                          arrowprops=dict(arrowstyle="-|>", color="purple",
                                          lw=1.0), zorder=4)
            px += world_dx
            py += world_dy
        pt += vtheta

    # Heading arrows + radius circles at every controller waypoint
    for idx, (x, y, th) in enumerate(ctrl_positions):
        adx = arrow_len * math.cos(th)
        ady = arrow_len * math.sin(th)
        ax2.annotate("", xy=(x + adx, y + ady), xytext=(x, y),
                      arrowprops=dict(arrowstyle="->", color="orchid",
                                      lw=1.4), zorder=5)
        ax2.add_patch(plt.Circle((x, y), robot_r,
                                  fill=False, edgecolor="plum",
                                  linewidth=0.6, linestyle="--", alpha=0.35,
                                  zorder=2))
        ax2.annotate(str(idx), (x, y), textcoords="offset points",
                      xytext=(-6, 6), fontsize=6, color="purple", alpha=0.7)

    ax2.legend(loc="upper left", fontsize=8)

    # ── Key at bottom ───────────────────────────────────────
    fig.text(0.5, 0.01,
             "Blue arrows = heading (θ) at waypoint  |  "
             "Purple arrows = forward drive (vx)  |  "
             "Dotted red = inflated obstacle  |  Green circle = robot radius",
             ha="center", fontsize=8, color="gray")

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    out = "plan_viz.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved {out}")
    plt.show()

if __name__ == "__main__":
    main()
