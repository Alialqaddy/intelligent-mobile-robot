#!/usr/bin/env python3
# handgesture.py  — Hand gestures + control session

import os, math, time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls

# ==== MediaPipe fix ====
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION_VERSION"] = "3"

try:
    import mediapipe as mp
    print("[handgesture] MediaPipe IMPORT OK")
except ImportError:
    print("[handgesture] ERROR: mediapipe is not installed inside mp-env")
    raise SystemExit

# ==== General constants ====
WIDTH, HEIGHT = 820, 616
DEBUG_WINDOW = "HandGesture Debug"

GESTURE_THUMBS_UP   = "thumbs_up"
GESTURE_THUMBS_DOWN = "thumbs_down"
GESTURE_OPEN_PALM   = "open_palm"      # exists but we won't use it as a command
GESTURE_CUSTOM_STOP = "custom_stop"    # the new gesture (thumb + two fingers)

# Photos directory
HOME = Path.home()
PHOTOS_DIR = HOME / "Pictures" / "robot_photos"
PHOTOS_DIR.mkdir(parents=True, exist_ok=True)

# ==== MediaPipe Hands ====
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils
hands    = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.3,
    min_tracking_confidence=0.3,
)

# ================== Gesture algorithms ==================

def landmarks_to_numpy(hand_landmarks, w, h):
    return np.array(
        [(lm.x * w, lm.y * h) for lm in hand_landmarks.landmark],
        dtype=np.float32,
    )

def is_finger_extended(pts, tip, pip, wrist=0, ratio=1.1):
    if np.linalg.norm(pts[pip] - pts[wrist]) < 1e-3:
        return False
    return np.linalg.norm(pts[tip] - pts[wrist]) > np.linalg.norm(pts[pip] - pts[wrist]) * ratio

def classify_states(pts):
    return {
        "thumb":  is_finger_extended(pts, 4, 3, ratio=1.05),
        "index":  is_finger_extended(pts, 8, 6),
        "middle": is_finger_extended(pts, 12, 10),
        "ring":   is_finger_extended(pts, 16, 14),
        "pinky":  is_finger_extended(pts, 20, 18),
    }

def thumb_angle(pts):
    v = pts[4] - pts[2]
    if np.linalg.norm(v) < 1e-3:
        return None
    return math.degrees(math.atan2(v[1], v[0]))  # 0 right, +90 down

def classify_gesture(pts):
    s = classify_states(pts)

    thumb  = s["thumb"]
    index  = s["index"]
    middle = s["middle"]
    ring   = s["ring"]
    pinky  = s["pinky"]

    # Count how many fingers (excluding thumb) are extended
    non_thumb_extended = sum([
        1 if index  else 0,
        1 if middle else 0,
        1 if ring   else 0,
        1 if pinky  else 0,
    ])

    # 🛑 New stop gesture:
    # thumb extended + at least two other fingers extended
    # but not all five (so it doesn't get confused with the old open palm)
    if thumb and 2 <= non_thumb_extended <= 3:
        return GESTURE_CUSTOM_STOP

    # ⚠️ Open palm is not used as a command now:
    # if thumb and index and middle and ring and pinky:
    #     return GESTURE_OPEN_PALM

    # 👍 / 👎 : thumb only
    if thumb and non_thumb_extended == 0:
        ang = thumb_angle(pts)
        if ang is None:
            return None

        # Same angles that were working for you:
        if -150 < ang < -30:
            return GESTURE_THUMBS_DOWN
        if 30 < ang < 150:
            return GESTURE_THUMBS_UP

    return None

def detect_gesture(frame_bgr):
    """Analyze one frame and return the gesture type (or None) + a frame with drawings."""
    h, w = frame_bgr.shape[:2]
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    result = hands.process(frame_rgb)

    gesture = None
    debug_frame = frame_bgr.copy()

    if result.multi_hand_landmarks:
        lm = result.multi_hand_landmarks[0]
        pts = landmarks_to_numpy(lm, w, h)
        gesture = classify_gesture(pts)
        mp_draw.draw_landmarks(debug_frame, lm, mp_hands.HAND_CONNECTIONS)

    label = gesture if gesture else "None"
    cv2.putText(debug_frame, f"Gesture: {label}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
    return gesture, debug_frame

# ================== Gesture control session ==================

def run_hand_gesture_session():
    """
    Opens the camera and waits for hand commands until:
      - 👍  : takes a photo and saves it, and stays in waiting mode.
      - 👎  : returns 'back'  -> used to send the robot back.
      - the new gesture (thumb + two fingers): returns 'stop' -> no return.
    Returns at the end: "back" or "stop".
    """
    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
    )
    cam.configure(cfg)

    cam.set_controls({   # only safe stuff
        "AwbEnable": True,
        "AwbMode": controls.AwbModeEnum.Auto,
        "Brightness": 0.0,
        "Contrast": 1.2,
        "Saturation": 1.2,
    })

    cam.start()
    time.sleep(0.3)

    print("\n[handgesture] Hand control mode is running:")
    print("  👍  : take a photo")
    print("  👎  : return home")
    print("  ✋ (new gesture: thumb + two fingers) : stop at this point (no return)")
    print("  ESC : manual exit (for debugging)\n")

    decision = "stop"       # default if we don't see any clear command
    last_gesture = None
    streak = 0

    try:
        while True:
            frame_rgb = cam.capture_array()
            # Rotate 180° depending on camera mounting
            frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            gesture, debug_frame = detect_gesture(frame_bgr)

            # Show window
            cv2.imshow(DEBUG_WINDOW, debug_frame)

            # Track gesture persistence (to reduce noise)
            if gesture == last_gesture and gesture is not None:
                streak += 1
            else:
                last_gesture = gesture
                streak = 1

            # 👍  — take a photo (do not exit the session)
            if gesture == GESTURE_THUMBS_UP and streak >= 5:
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = PHOTOS_DIR / f"shot_{ts}.jpg"
                cv2.imwrite(str(path), frame_bgr)
                print(f"[handgesture] 📸 Saved photo: {path}")
                streak = 0  # go back to waiting for a new gesture

            # 👎  — return
            if gesture == GESTURE_THUMBS_DOWN and streak >= 5:
                print("[handgesture] 👎  Return-home command")
                decision = "back"
                break

            # New gesture — stop here
            if gesture == GESTURE_CUSTOM_STOP and streak >= 5:
                print("[handgesture] ✋  Stop command (no return)")
                decision = "stop"
                break

            # ESC for manual exit (testing)
            if cv2.waitKey(1) & 0xFF == 27:
                print("[handgesture] ESC: exiting hand mode")
                break

    finally:
        cam.stop()
        cv2.destroyAllWindows()

    return decision

# Standalone run for testing
if __name__ == "__main__":
    dec = run_hand_gesture_session()
    print("Session decision =", dec)