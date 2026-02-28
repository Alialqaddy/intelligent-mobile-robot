#!/usr/bin/env python3
import csv
import math
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

HOME      = Path.home()
RUNS_DIR  = HOME / "runs"
PATHS_DIR = HOME / "paths"

PATHS_DIR.mkdir(parents=True, exist_ok=True)

# ==== Same calibration exactly like path_recorder ====
M_PER_TICK = 0.014   # meters per tick
BASELINE   = 0.31    # base width in meters

# Waypoint spacing (change if you want)
MIN_WAYPOINT_DIST = 0.20  # roughly every 20 cm


def pick_latest_csv():
    runs = sorted(RUNS_DIR.glob("run_*.csv"), key=lambda f: f.stat().st_mtime)
    if not runs:
        raise SystemExit("❌ No run_*.csv found in the runs folder")
    return runs[-1]


def compute_odometry(csv_path):
    """
    Compute x,y,theta from a CSV file
    with a small trick:
      - if cmd_left and cmd_right are almost equal and have the same sign,
        we treat the motion as straight (dtheta = 0) even if wheel ticks differ a bit.
      - otherwise we use the classic formula (wheel difference / BASELINE).
    """
    xs = []
    ys = []
    thetas = []
    ts = []

    x = 0.0
    y = 0.0
    theta = 0.0

    prev_c1 = None
    prev_c2 = None

    # Threshold to decide that wheel commands are "similar" => straight
    STRAIGHT_CMD_EPS = 0.12  # try 0.1 to 0.2 depending on what feels right

    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise SystemExit("❌ CSV file is empty!")

    for row in rows:
        try:
            t    = float(row["t"])
            c1   = int(row["count1"])
            c2   = int(row["count2"])
            cmdL = float(row.get("cmd_left",  "0"))
            cmdR = float(row.get("cmd_right", "0"))
        except (KeyError, ValueError):
            continue

        # First row: only store the initial state
        if prev_c1 is None:
            prev_c1 = c1
            prev_c2 = c2
            xs.append(x)
            ys.append(y)
            thetas.append(theta)
            ts.append(t)
            continue

        dc1 = c1 - prev_c1
        dc2 = c2 - prev_c2
        prev_c1 = c1
        prev_c2 = c2

        # Sign from commands only (same as path_recorder)
        if cmdL > 0:
            sign_L = +1.0
        elif cmdL < 0:
            sign_L = -1.0
        else:
            sign_L = 0.0

        if cmdR > 0:
            sign_R = +1.0
        elif cmdR < 0:
            sign_R = -1.0
        else:
            sign_R = 0.0

        # No commands and no ticks -> standing still
        if (sign_L == 0.0 and sign_R == 0.0) or (dc1 == 0 and dc2 == 0):
            xs.append(x)
            ys.append(y)
            thetas.append(theta)
            ts.append(t)
            continue

        # Distance per wheel
        dL = dc1 * sign_L * M_PER_TICK
        dR = dc2 * sign_R * M_PER_TICK

        # Is this segment "straight" based on commands?
        same_sign = (sign_L == sign_R) and (sign_L != 0.0)
        cmds_close = abs(cmdL - cmdR) < STRAIGHT_CMD_EPS

        if same_sign and cmds_close:
            # Straight segment: ignore small wheel differences
            ds = 0.5 * (dL + dR)
            dtheta = 0.0
        else:
            # Real turning/curving segment
            ds = 0.5 * (dL + dR)
            dtheta = (dR - dL) / BASELINE if BASELINE != 0 else 0.0

        theta_mid = theta + 0.5 * dtheta
        x += ds * math.cos(theta_mid)
        y += ds * math.sin(theta_mid)
        theta += dtheta

        xs.append(x)
        ys.append(y)
        thetas.append(theta)
        ts.append(t)

    return np.array(xs), np.array(ys), np.array(thetas)


def wrap_angle(a: float) -> float:
    """Return angle within [-pi, +pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def downsample_to_waypoints(xs, ys,
                            min_dist=0.20,
                            angle_thresh_deg=25.0):
    """
    Create waypoints from the full trajectory:
      - add the first point.
      - whenever we get min_dist away from the last waypoint, add a new one.
      - or if the heading changes by more than angle_thresh_deg,
        add a waypoint even if distance is small (at corners).
      - finally ensure the last point is included.
    """
    if len(xs) < 2:
        return np.array(xs), np.array(ys)

    w_x = [xs[0]]
    w_y = [ys[0]]
    last_x = xs[0]
    last_y = ys[0]

    prev_heading = None

    for x, y in zip(xs[1:], ys[1:]):
        dx = x - last_x
        dy = y - last_y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            continue

        heading = math.atan2(dy, dx)

        if prev_heading is None:
            angle_change = 0.0
        else:
            angle_change = abs(math.degrees(wrap_angle(heading - prev_heading)))

        # Distance or sharp turn condition
        if (dist >= min_dist) or (angle_change >= angle_thresh_deg):
            w_x.append(x)
            w_y.append(y)
            last_x = x
            last_y = y
            prev_heading = heading

    # Ensure we add the last point exactly
    if w_x[-1] != xs[-1] or w_y[-1] != ys[-1]:
        w_x.append(xs[-1])
        w_y.append(ys[-1])

    return np.array(w_x), np.array(w_y)


def save_path_csv(xs, ys, path_file):
    """Save only the waypoints (used by target_follower)."""
    with open(path_file, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for x, y in zip(xs, ys):
            w.writerow([f"{x:.6f}", f"{y:.6f}"])


def main():
    csv_path = pick_latest_csv()
    print("Using run:", csv_path)

    xs, ys, thetas = compute_odometry(csv_path)

    L = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    print(f"Original path (full) length ≈ {L:.3f} m")
    print(f"Final heading ≈ {math.degrees(thetas[-1]):.1f} deg")

    # Generate waypoints from the same path without changing it
    w_x, w_y = downsample_to_waypoints(
        xs, ys,
        min_dist=MIN_WAYPOINT_DIST,
        angle_thresh_deg=25.0  # adjust if you want smaller/larger corner sensitivity
    )
    print(f"Waypoints count: {len(w_x)}")

    # Choose a name for the path
    default_name = datetime.now().strftime("path_%Y%m%d_%H%M%S")
    name = input(f"Enter a name for the path (without .csv) [{default_name}]: ").strip()
    if not name:
        name = default_name

    path_file = PATHS_DIR / f"{name}.csv"
    save_path_csv(w_x, w_y, path_file)
    print("Saved waypoints path to:", path_file)

    # Plot for review: full path + waypoints
    plt.figure()
    plt.plot(xs, ys, "b.-", markersize=2, label="Full odom trajectory")
    plt.plot(w_x, w_y, "ro-", markersize=4, label="Waypoints (~20cm + corners)")
    plt.scatter(xs[0], ys[0], c="g", label="Start")
    plt.scatter(xs[-1], ys[-1], c="k", label="End")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Extracted Path (full) + Waypoints")
    plt.show()


if __name__ == "__main__":
    main()