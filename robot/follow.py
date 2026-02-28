#!/usr/bin/env python3
import csv
import math
import time
from pathlib import Path

from gpiozero import Device, Button
from gpiozero.pins.lgpio import LGPIOFactory
Device.pin_factory = LGPIOFactory()

from gpiozero import DistanceSensor  # ==== NEW: distance sensor ====
from motor_control import forward, backward, spin_left, spin_right, stop_all
from handgesture import run_hand_gesture_session   # ← We only added this line

# ================= General settings =================
HOME      = Path.home()
PATHS_DIR = HOME / "paths"     # path files
PATHS_DIR.mkdir(parents=True, exist_ok=True)

# Odometry calibration (same as extract_path / driving)
M_PER_TICK = 0.014
BASELINE   = 0.31

# Encoders
ENC1_PIN = 5
ENC2_PIN = 25

count1 = 0
count2 = 0

# Motion
DRIVE_SPEED      = 0.3      # driving speed on straight segments
TURN_SPEED       = 0.35     # turning speed
ANGLE_MARGIN_RAD = math.radians(3.0)   # allow ~3 degrees error in turning
MIN_SEG_DIST     = 0.02                # ignore any segment shorter than 2 cm (noise)

# ==== NEW: distance sensor settings ====
DIST_TRIG_PIN     = 24   # change according to your actual wiring
DIST_ECHO_PIN     = 26
DIST_SLOW_M       = 0.40  # starts slowing down if less than 40 cm
DIST_STOP_M       = 0.30  # stops roughly at 30 cm
distance_sensor   = None  # will be initialized in setup_distance_sensor()


# ================= Encoders =================
def _enc1_pressed():
    global count1
    count1 += 1

def _enc2_pressed():
    global count2
    count2 += 1

def setup_encoders():
    """Initialize encoder buttons and assign callbacks correctly."""
    global enc1_btn, enc2_btn
    enc1_btn = Button(ENC1_PIN, pull_up=True, bounce_time=0.001)
    enc2_btn = Button(ENC2_PIN, pull_up=True, bounce_time=0.001)
    enc1_btn.when_pressed = _enc1_pressed
    enc2_btn.when_pressed = _enc2_pressed

def cleanup_encoders():
    """Release callbacks (for safety)."""
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

# ==== NEW: distance sensor setup ====
def setup_distance_sensor():
    """Initialize the distance sensor (Ultrasonic)."""
    global distance_sensor
    try:
        distance_sensor = DistanceSensor(
            echo=DIST_ECHO_PIN,
            trigger=DIST_TRIG_PIN,
            max_distance=1.5,   # 1.5 meters max
            threshold_distance=0.3,  # not very important here
        )
        print("[distance] DistanceSensor initialized.")
    except Exception as e:
        distance_sensor = None
        print(f"[distance] WARNING: failed to init distance sensor: {e}")


# ================= Helper functions =================
def wrap_angle(a):
    """Return angle within [-pi, +pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def drive_distance(dist_m: float, speed: float = DRIVE_SPEED):
    """
    Drive a distance in meters using encoders.
    dist_m positive for forward, negative for backward.
    We slow down near the target to reduce overshoot.
    + NEW: during motion, if the distance sensor sees an obstacle:
           - < 40 cm -> drive at slow speed
           - < 30 cm -> stop, print a message, wait until obstacle clears, then continue.
    """
    global count1, count2

    if abs(dist_m) < 1e-3:
        return

    direction = 1.0 if dist_m >= 0 else -1.0
    target = abs(dist_m)

    fast_speed = max(0.3, min(1.0, speed))
    slow_speed = max(0.3, fast_speed * 0.4)

    start_c1 = count1
    start_c2 = count2

    # Start moving
    if direction > 0:
        forward(fast_speed)
    else:
        backward(fast_speed)

    try:
        while True:
            # ==== NEW: read distance sensor during motion ====
            obstacle_m = None
            if distance_sensor is not None:
                try:
                    obstacle_m = distance_sensor.distance  # roughly in meters
                except Exception:
                    obstacle_m = None

            # If we have a reasonable reading
            if obstacle_m is not None and 0.02 < obstacle_m < 1.5:
                # Too close -> stop and wait
                if obstacle_m <= DIST_STOP_M:
                    safe_stop()
                    print(f"[distance] Obstacle too close at ~{obstacle_m*100:.0f} cm, pausing.")
                    # Wait until obstacle clears (becomes > DIST_SLOW_M)
                    while True:
                        time.sleep(0.1)
                        try:
                            d2 = distance_sensor.distance
                        except Exception:
                            d2 = None

                        if d2 is not None and d2 > DIST_SLOW_M:
                            print("[distance] Obstacle cleared, resuming path...")
                            # Resume with slow speed initially
                            if direction > 0:
                                forward(slow_speed)
                            else:
                                backward(slow_speed)
                            break

                # Between 30 and 40 cm -> slow speed
                elif obstacle_m <= DIST_SLOW_M:
                    if direction > 0:
                        forward(slow_speed)
                    else:
                        backward(slow_speed)
                # Farther than 40 cm -> back to normal speed (unless close to target)
                else:
                    if direction > 0:
                        forward(fast_speed)
                    else:
                        backward(fast_speed)
            # ==== END NEW obstacle handling ====

            c1 = count1
            c2 = count2

            d1 = (c1 - start_c1) * M_PER_TICK
            d2 = (c2 - start_c2) * M_PER_TICK
            ds = 0.5 * (d1 + d2)
            traveled = abs(ds)

            remaining = target - traveled

            # Slow down in the last ~15 cm
            if 0.0 < remaining < 0.15:
                if direction > 0:
                    forward(slow_speed)
                else:
                    backward(slow_speed)

            if traveled >= target:
                break

            time.sleep(0.005)

    finally:
        safe_stop()
        time.sleep(0.05)


def turn_angle(angle_rad: float, speed: float = TURN_SPEED):
    """
    Turn by angle_rad radians using encoders.
    + angle = left turn, - angle = right turn.
    """
    global count1, count2

    angle_rad = wrap_angle(angle_rad)
    if abs(angle_rad) < math.radians(1.0):
        return

    target = abs(angle_rad)

    start_c1 = count1
    start_c2 = count2

    if angle_rad > 0:
        # Left: left backward, right forward
        spin_left(speed)
        sign_L, sign_R = -1.0, +1.0
    else:
        # Right
        spin_right(speed)
        sign_L, sign_R = +1.0, -1.0

    try:
        while True:
            c1 = count1
            c2 = count2

            ds_L = (c1 - start_c1) * M_PER_TICK * sign_L
            ds_R = (c2 - start_c2) * M_PER_TICK * sign_R
            dtheta = (ds_R - ds_L) / BASELINE if BASELINE != 0 else 0.0
            dtheta_abs = abs(dtheta)

            # Stop when we approximately reach the desired angle (with margin)
            if dtheta_abs >= max(0.0, target - ANGLE_MARGIN_RAD):
                break

            time.sleep(0.01)

    finally:
        safe_stop()
        time.sleep(0.05)


# ================= Read/choose path file =================
def list_path_files():
    files = sorted(PATHS_DIR.glob("*.csv"))
    return files

def choose_path_file():
    files = list_path_files()
    if not files:
        raise SystemExit(f"❌ No path files found inside {PATHS_DIR}")

    print("\nAvailable paths (from paths/ folder):")
    for i, f in enumerate(files, start=1):
        print(f"  {i}: {f.name}")

    while True:
        choice = input("\nChoose path number (or q to quit): ").strip()
        if choice.lower() == "q":
            raise SystemExit("Cancelled by user.")
        try:
            idx = int(choice)
            if 1 <= idx <= len(files):
                return files[idx - 1]
        except ValueError:
            pass
        print("⚠️ Invalid input, try again.")


def load_path_points(path_file: Path):
    if not path_file.exists():
        raise SystemExit(f"❌ Path file not found: {path_file}")

    xs = []
    ys = []
    with open(path_file, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row["x"])
                y = float(row["y"])
                xs.append(x)
                ys.append(y)
            except (KeyError, ValueError):
                continue

    if not xs:
        raise SystemExit("❌ Path file is empty or invalid")

    return xs, ys


# ================= Main follow for a given path =================
def follow_path_points(xs, ys, initial_theta=0.0):
    """
    Follow a list of (x,y) points sequentially.
    Returns the final heading angle (sum of all turns performed).
    """
    curr_theta = initial_theta

    if len(xs) < 2:
        print("Path contains fewer than two points; nothing to follow.")
        return curr_theta

    for i in range(1, len(xs)):
        x_prev, y_prev = xs[i-1], ys[i-1]
        x_curr, y_curr = xs[i], ys[i]

        dx = x_curr - x_prev
        dy = y_curr - y_prev
        seg_d = math.hypot(dx, dy)

        if seg_d < MIN_SEG_DIST:
            continue

        desired_heading = math.atan2(dy, dx)
        dtheta = wrap_angle(desired_heading - curr_theta)

        print(f"\nSegment {i}/{len(xs)-1}")
        print(f"  From ({x_prev:.3f},{y_prev:.3f}) to ({x_curr:.3f},{y_curr:.3f})")
        print(f"  distance ≈ {seg_d:.3f} m , turn ≈ {math.degrees(dtheta):.1f} deg")

        # 1) Turn the required angle
        turn_angle(dtheta, TURN_SPEED)
        curr_theta = wrap_angle(curr_theta + dtheta)  # update heading based on the executed turn

        # 2) Drive the distance (distance sensor is active inside drive_distance)
        drive_distance(seg_d, DRIVE_SPEED)

    print("\n✅ Finished this path.")
    return curr_theta


# ================= MAIN =================
def main():
    path_file = choose_path_file()
    print(f"\nFollowing path from file: {path_file}")

    xs, ys = load_path_points(path_file)
    print(f"Loaded {len(xs)} points in path.")

    print("\nPlace the robot roughly at the same start point of the path and facing the same direction (θ≈0),")
    input("then press Enter to start...")

    global count1, count2
    count1 = 0
    count2 = 0

    setup_encoders()
    setup_distance_sensor()   # ==== NEW: initialize distance sensor ====

    try:
        # 1) Go from start to end
        theta_end = follow_path_points(xs, ys, initial_theta=0.0)
        print(f"\nEstimated heading at the end of the path (forward): {math.degrees(theta_end):.1f} deg")

        # 2) After reaching -> hand-gesture control for the return
        print("\n🤚 Entered hand-control mode at the end point.")
        print("   👍  = take a photo (stay in the same mode)")
        print("   👎  = return to the start point")
        print("   ✋ (new gesture) = stop and stay in place")

        decision = run_hand_gesture_session()   # returns 'back' or 'stop'

        if decision == "back":
            print("\n🔁 Before returning:")
            print("   Rotate the robot manually so it faces the first segment on the way back (toward the start point).")
            print("   Roughly point its front in the opposite direction of the last path segment.")
            input("   When done, press Enter to start returning...")

            # Compute the heading of the first segment in the return trip (from last point to the previous one)
            xs_back = list(reversed(xs))
            ys_back = list(reversed(ys))

            if len(xs_back) >= 2:
                dx0 = xs_back[1] - xs_back[0]
                dy0 = ys_back[1] - ys_back[0]
                initial_back_heading = math.atan2(dy0, dx0)
            else:
                initial_back_heading = 0.0

            # 3) Return along the same path without an automatic 180 turn
            theta_back_end = follow_path_points(xs_back, ys_back, initial_theta=initial_back_heading)
            print(f"\nEstimated heading after returning: {math.degrees(theta_back_end):.1f} deg")

            # 4) At the end: zero the heading (small correction turn)
            print("\n⚙️ Zeroing the final heading based on odometry...")
            turn_angle(-theta_back_end, TURN_SPEED)
            print("✅ Attempted heading zeroing (θ ≈ 0).")

        else:
            # decision == "stop"
            print("\n🛑 Chose to stop at the end point. Will not return on the path.")

        print("\n✨ Program finished.")
    finally:
        safe_stop()
        cleanup_encoders()


if __name__ == "__main__":
    main()