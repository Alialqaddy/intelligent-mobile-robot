# All settings in one place

# ==== GPIO (BCM) ====
IN1, IN2 = 17, 27      # L298N - Left motor
IN3, IN4 = 22, 23      # L298N - Right motor
ENA, ENB = 12, 13      # L298N - PWM enables

TRIG, ECHO = 23, 26    # HC-SR04 (Echo via a voltage divider 5V->3.3V)

# ==== Speeds ====
BASE_MAX = 0.60        # maximum forward speed (0..1)
BASE_MIN = 0.25        # start speed (smooth acceleration)
TURN_SPEED = 0.70      # in-place turning speed

# ==== Behavior ====
SLOWDOWN_START_CM = 100.0  # start slowing down from 1 meter
STOP_CM = 25.0             # stop at 25 cm
TARGET_CLEAR_CM = 100.0    # when scanning, look for ≥1m
RAMP_UP_TIME = 1.2         # acceleration time from MIN to MAX
CHECK_PERIOD = 0.04

# ==== Scan (360°) ====
SCAN_STEPS = 14            # readings during a full rotation
ROTATE_360_TIME = 2.20     # adjust after rotation calibration
SCAN_SETTLE = 0.08         # small wait before reading
TURN_NUDGE = 0.12          # small forward nudge after choosing a direction

# ==== Misc ====
SWAP_SIDES = True # if your wiring is flipped left/right 
# ENC1 leftwheels  ENC2 rightwheels
# 0.012 m per tick