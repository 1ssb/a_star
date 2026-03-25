#!/usr/bin/env python3
"""
visualize.py — Socket-based visualizer for the SE(2) A* planner.

Listens on a TCP port (default 9876) for a JSON payload from planner,
then plots waypoints, controller trajectory, and obstacles.

Usage:  python3 visualize.py [--port 9876]

The planner (./planner) auto-connects and streams results.
"""

import json
import math
import socket
import struct
import sys

import matplotlib
if "--save-only" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as mtransforms
import numpy as np


def receive_json(port: int) -> dict:
    """Listen on TCP port, accept one connection, read length-prefixed JSON."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print(f"Visualizer listening on port {port} ...")
    conn, addr = srv.accept()
    print(f"Connected from {addr}")

    # Read 4-byte big-endian length
    raw_len = b""
    while len(raw_len) < 4:
        chunk = conn.recv(4 - len(raw_len))
        if not chunk:
            raise RuntimeError("Connection closed before length header")
        raw_len += chunk
    msg_len = struct.unpack("!I", raw_len)[0]

    # Read message
    data = b""
    while len(data) < msg_len:
        chunk = conn.recv(min(65536, msg_len - len(data)))
        if not chunk:
            raise RuntimeError("Connection closed during message")
        data += chunk

    conn.close()
    srv.close()
    return json.loads(data.decode("utf-8"))


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
    port = 9876
    out_file = "plan_viz.png"
    show = True
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--port" and i + 1 < len(args):
            port = int(args[i + 1]); i += 2
        elif args[i] == "--out" and i + 1 < len(args):
            out_file = args[i + 1]; i += 2
        elif args[i] == "--save-only":
            show = False; i += 1
        else:
            i += 1

    data = receive_json(port)

    cfg = data["config"]
    obstacles = data["obstacles"]
    wp = np.array(data["waypoints"])       # [[x, y, theta], ...]
    ctrls = data["controls"]               # [[vx, vy, vtheta, duration], ...]

    robot_r = cfg["robot_radius"]
    safety = cfg["safety_margin"]
    arrow_len = robot_r * 0.6

    # ── Figure ──
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle("SE(2) A* Planner — Waypoints & Controller",
                 fontsize=14, fontweight="bold")

    draw_panel(ax1, "A* Waypoints", cfg, obstacles, wp, robot_r, safety)
    draw_panel(ax2, "Controller Commands", cfg, obstacles, wp, robot_r, safety)

    # ── Left panel: waypoints ──
    ax1.plot(wp[:, 0], wp[:, 1], "b.-", linewidth=1.2, markersize=6,
             alpha=0.8, label="Path", zorder=3)
    for i, (x, y, th) in enumerate(wp):
        dx = arrow_len * math.cos(th)
        dy = arrow_len * math.sin(th)
        ax1.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                      arrowprops=dict(arrowstyle="->", color="dodgerblue",
                                      lw=1.4), zorder=5)
        ax1.add_patch(plt.Circle((x, y), robot_r, fill=False,
                                  edgecolor="cornflowerblue",
                                  linewidth=0.6, linestyle="--", alpha=0.35,
                                  zorder=2))
        ax1.annotate(str(i), (x, y), textcoords="offset points",
                      xytext=(-6, 6), fontsize=6, color="navy", alpha=0.8)
    ax1.legend(loc="upper left", fontsize=8)

    # ── Right panel: controller replay ──
    # Replay velocity commands: vx * duration = forward displacement
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
