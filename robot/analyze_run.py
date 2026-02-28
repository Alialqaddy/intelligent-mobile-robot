#!/usr/bin/env python3
import csv
import math
import os
import sys
import glob
import matplotlib.pyplot as plt

# Odometry constants
M_PER_TICK = 0.0103   # meters per tick
BASE_B     = 0.17    # meters (base width)

def pick_latest_run(runs_dir: str):
    pattern = os.path.join(runs_dir, "run_*.csv")
    files = glob.glob(pattern)
    if not files:
        print(f"No run_*.csv files found inside {runs_dir}")
        sys.exit(1)
    files.sort()
    return files[-1]

def load_and_integrate(csv_path: str):
    xs = []
    ys = []
    thetas = []
    ts = []

    x = 0.0
    y = 0.0
    theta = 0.0

    prev_count1 = None
    prev_count2 = None

    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("The file is empty!")
        sys.exit(1)

    for row in rows:
        try:
            t   = float(row["t"])
            c1  = int(row["count1"])
            c2  = int(row["count2"])
            cmdL = float(row.get("cmd_left",  "0"))
            cmdR = float(row.get("cmd_right", "0"))
        except (KeyError, ValueError) as e:
            print("Skipped a row with an issue:", e)
            continue

        # First row: just store the initial state
        if prev_count1 is None:
            prev_count1 = c1
            prev_count2 = c2
            xs.append(x)
            ys.append(y)
            thetas.append(theta)
            ts.append(t)
            continue

        # Raw tick differences
        d1_raw = c1 - prev_count1
        d2_raw = c2 - prev_count2
        prev_count1 = c1
        prev_count2 = c2

        # Direction signs from velocity command
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

        dL_ticks = d1_raw * sign_L
        dR_ticks = d2_raw * sign_R

        # If there is no real movement, store the same point
        if dL_ticks == 0 and dR_ticks == 0:
            xs.append(x)
            ys.append(y)
            thetas.append(theta)
            ts.append(t)
            continue

        # Distances and heading update
        ds_L = dL_ticks * M_PER_TICK
        ds_R = dR_ticks * M_PER_TICK
        ds   = 0.5 * (ds_L + ds_R)
        dtheta = (ds_R - ds_L) / BASE_B if BASE_B != 0 else 0.0

        theta_mid = theta + 0.5 * dtheta
        x += ds * math.cos(theta_mid)
        y += ds * math.sin(theta_mid)
        theta += dtheta

        xs.append(x)
        ys.append(y)
        thetas.append(theta)
        ts.append(t)

    return ts, xs, ys, thetas

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    runs_dir = os.path.join(script_dir, "runs")

    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(script_dir, csv_path)
    else:
        csv_path = pick_latest_run(runs_dir)

    if not os.path.isfile(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    print(f"Using file: {csv_path}")

    ts, xs, ys, thetas = load_and_integrate(csv_path)

    print(f"Final position:")
    print(f"  x = {xs[-1]:.3f} m")
    print(f"  y = {ys[-1]:.3f} m")
    print(f"  theta = {thetas[-1]:.3f} rad  ≈ {thetas[-1] * 180/math.pi:.1f} deg")

    # Adjust zoom based on trajectory range
    max_range = max(max(abs(max(xs)), abs(min(xs)), 0.1),
                    max(abs(max(ys)), abs(min(ys)), 0.1))

    plt.figure()
    plt.plot(xs, ys, marker=".", linewidth=1)
    plt.scatter([xs[0]], [ys[0]], c="green", label="Start")
    plt.scatter([xs[-1]], [ys[-1]], c="red", label="End")
    plt.axis("equal")
    plt.xlim(-max_range, max_range)
    plt.ylim(-max_range, max_range)
    plt.grid(True)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(os.path.basename(csv_path))
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()