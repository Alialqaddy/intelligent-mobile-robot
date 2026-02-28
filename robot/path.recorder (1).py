#!/usr/bin/env python3
import time
import math
import csv
from datetime import datetime
from pathlib import Path

from gpiozero import Device, Button
from gpiozero.pins.lgpio import LGPIOFactory
Device.pin_factory = LGPIOFactory()

from motor_control import (
    forward,
    backward,
    spin_left,
    spin_right,
    stop_all,
)

# ============ General settings ============
HOME     = Path.home()
RUNS_DIR = HOME / "runs"
RUNS_DIR.mkdir(parents=True, exist_ok=True)

# Same calibration you were using (adjust if you changed it)
M_PER_TICK = 0.014   # meters per tick
BASELINE   = 0.31     # base width in meters

# Encoders
ENC1_PIN = 5
ENC2_PIN = 25

count1 = 0
count2 = 0

def _enc1_pressed():
    global count1
    count1 += 1

def _enc2_pressed():
    global count2
    count2 += 1

def setup_encoders():
    global enc1_btn, enc2_btn
    enc1_btn = Button(ENC1_PIN, pull_up=True, bounce_time=0.001)
    enc2_btn = Button(ENC2_PIN, pull_up=True, bounce_time=0.001)
    enc1_btn.when_pressed = _enc1_pressed
    enc2_btn.when_pressed = _enc2_pressed

def cleanup_encoders():
    try:
        enc1_btn.when_pressed = None
        enc2_btn.when_pressed = None
    except Exception:
        pass

def safe_stop():
    try:
        stop_all()
    except Exception:
        pass

def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

# ============ Step settings ============
STEP_DIST      = 0.20      # meters per straight pulse (roughly waypoint spacing)
STEP_ANGLE_DEG = 30.0      # degrees for A/D rotation if you want to use it later (not used here)
DRIVE_SPEED    = 0.40
TURN_SPEED     = 0.40

TICK           = 0.02      # update loop period for A/D
HOLD_TIMEOUT   = 0.10      # consider key as held for ~100ms

# =========================================================
#   Encoder-based motion functions (PULSED)
# =========================================================
def drive_distance(dist_m, writer, start_time):
    """
    Drive distance dist_m (positive forward, negative backward) using encoders.
    Logs to CSV from inside the function.
    """
    global count1, count2

    if abs(dist_m) < 1e-3:
        return

    direction = 1.0 if dist_m >= 0 else -1.0
    target    = abs(dist_m)

    fast_speed = DRIVE_SPEED
    slow_speed = max(0.25, DRIVE_SPEED * 0.4)

    start_c1 = count1
    start_c2 = count2

    # Initial start
    if direction > 0:
        forward(fast_speed)
        cmd_L = cmd_R = +fast_speed
    else:
        backward(fast_speed)
        cmd_L = cmd_R = -fast_speed

    try:
        while True:
            now = time.time()
            c1  = count1
            c2  = count2

            d1 = (c1 - start_c1) * M_PER_TICK
            d2 = (c2 - start_c2) * M_PER_TICK
            ds = 0.5 * (d1 + d2)
            traveled  = abs(ds)
            remaining = target - traveled

            # Slow down near the target
            if 0.0 < remaining < 0.15:
                if direction > 0:
                    forward(slow_speed)
                    cmd_L = cmd_R = +slow_speed
                else:
                    backward(slow_speed)
                    cmd_L = cmd_R = -slow_speed

            # Write a CSV row
            t_rel = now - start_time
            writer.writerow([
                f"{t_rel:.6f}",
                c1,
                c2,
                f"{cmd_L:.3f}",
                f"{cmd_R:.3f}",
            ])

            if traveled >= target:
                break

            time.sleep(0.01)
    finally:
        safe_stop()
        # Stop row
        now = time.time()
        t_rel = now - start_time
        writer.writerow([f"{t_rel:.6f}", count1, count2, "0.000", "0.000"])
        time.sleep(0.05)


def turn_angle(angle_rad, writer, start_time):
    """
    Turn by angle_rad (radians).
    + = left, - = right. Fully based on encoder difference.
    """
    global count1, count2

    angle_rad = wrap_angle(angle_rad)
    if abs(angle_rad) < math.radians(1):
        return

    start_c1 = count1
    start_c2 = count2

    if angle_rad > 0:
        # Left: left backward, right forward
        spin_left(TURN_SPEED)
        sign_L, sign_R = -1.0, +1.0
        cmd_L, cmd_R   = -TURN_SPEED, +TURN_SPEED
    else:
        # Right
        spin_right(TURN_SPEED)
        sign_L, sign_R = +1.0, -1.0
        cmd_L, cmd_R   = +TURN_SPEED, -TURN_SPEED

    target = abs(angle_rad)

    try:
        while True:
            now = time.time()
            c1  = count1
            c2  = count2

            ds_L = (c1 - start_c1) * M_PER_TICK * sign_L
            ds_R = (c2 - start_c2) * M_PER_TICK * sign_R
            dtheta = (ds_R - ds_L) / BASELINE if BASELINE != 0 else 0.0
            progress = abs(dtheta)

            # Log
            t_rel = now - start_time
            writer.writerow([
                f"{t_rel:.6f}",
                c1,
                c2,
                f"{cmd_L:.3f}",
                f"{cmd_R:.3f}",
            ])

            if progress >= target:
                break

            time.sleep(0.01)
    finally:
        safe_stop()
        now = time.time()
        t_rel = now - start_time
        writer.writerow([f"{t_rel:.6f}", count1, count2, "0.000", "0.000"])
        time.sleep(0.05)


# =========================================================
#   Main program: pulsed training + continuous A/D turning
# =========================================================
def main():
    import curses

    setup_encoders()

    # Prepare CSV file
    ts_name  = datetime.now().strftime("run_%Y%m%d_%H%M%S.csv")
    csv_path = RUNS_DIR / ts_name
    csv_file = open(csv_path, "w", newline="", encoding="utf-8")
    writer   = csv.writer(csv_file)
    writer.writerow(["t", "count1", "count2", "cmd_left", "cmd_right"])
    start_time = time.time()

    def curses_main(stdscr):
        curses.cbreak()
        stdscr.nodelay(True)   # non-blocking for A/D
        stdscr.keypad(True)

        stdscr.clear()
        stdscr.addstr(
            0, 0,
            "Pulsed + Free Turn Training\n"
            "W: +%.2f m | S: -%.2f m\n"
            "Z: +90 deg (left) | X: -90 deg (right) | C: 180 deg turn\n"
            "A/D: continuous spin (hold): left/right\n"
            "ESC/Q: Exit\n" % (STEP_DIST, STEP_DIST)
        )
        stdscr.refresh()

        last_a_time = 0.0
        last_d_time = 0.0

        mode = "idle"
        cmd_L = 0.0
        cmd_R = 0.0

        try:
            while True:
                now = time.time()
                ch = stdscr.getch()

                # ---------- Key handling ----------
                if ch != -1:
                    if ch in (27, ord('q'), ord('Q')):  # ESC or q
                        break

                    elif ch in (ord('w'), ord('W')):
                        # Straight forward pulse (PULSED)
                        drive_distance(+STEP_DIST, writer, start_time)

                    elif ch in (ord('s'), ord('S')):
                        # Straight backward pulse (PULSED)
                        drive_distance(-STEP_DIST, writer, start_time)

                    elif ch in (ord('z'), ord('Z')):
                        # 90 degrees left
                        turn_angle(math.radians(+90.0), writer, start_time)

                    elif ch in (ord('x'), ord('X')):
                        # 90 degrees right
                        turn_angle(math.radians(-90.0), writer, start_time)

                    elif ch in (ord('c'), ord('C')):
                        # 180 degrees (turns and reverses direction)
                        turn_angle(math.radians(180.0), writer, start_time)

                    elif ch in (ord('a'), ord('A')):
                        last_a_time = now

                    elif ch in (ord('d'), ord('D')):
                        last_d_time = now

                # ---------- Continuous A / D hold state ----------
                holding_a = (now - last_a_time) <= HOLD_TIMEOUT
                holding_d = (now - last_d_time) <= HOLD_TIMEOUT

                if holding_a and not holding_d:
                    mode = "spin_left"
                elif holding_d and not holding_a:
                    mode = "spin_right"
                else:
                    mode = "idle"

                if mode == "spin_left":
                    spin_left(TURN_SPEED)
                    cmd_L, cmd_R = -TURN_SPEED, +TURN_SPEED
                elif mode == "spin_right":
                    spin_right(TURN_SPEED)
                    cmd_L, cmd_R = +TURN_SPEED, -TURN_SPEED
                else:
                    safe_stop()
                    cmd_L, cmd_R = 0.0, 0.0

                # ---------- Continuous logging for A/D/Idle state ----------
                t_rel = now - start_time
                writer.writerow([
                    f"{t_rel:.6f}",
                    count1,
                    count2,
                    f"{cmd_L:.3f}",
                    f"{cmd_R:.3f}",
                ])

                # Simple display
                stdscr.addstr(6, 0, f"Counts: L={count1:6d}  R={count2:6d}          ")
                stdscr.refresh()

                time.sleep(TICK)

        finally:
            safe_stop()

    try:
        import curses
        curses.wrapper(curses_main)
    finally:
        safe_stop()
        cleanup_encoders()
        csv_file.close()
        print(f"\nSaved CSV to: {csv_path}")
        print(f"Final counts: L={count1}, R={count2}")

if __name__ == "__main__":
    main()