#!/usr/bin/env python3
# simple_recorder.py
from picamera2 import Picamera2
import cv2, time
import numpy as np
from pathlib import Path
from datetime import datetime

# -------- Settings (taken from your code) --------
SHOW_VIEW =True           # True: show preview window, False: no window
WIDTH, HEIGHT = 820, 616  # camera resolution
FPS = 10                    # target recording FPS
OUTPUT_DIR = Path.home() / "Videos"   # save location
VIDEO_EXT = ".mp4"          # preferred container
FOURCC = "mp4v"             # codec (fallback handled below)
# -------------------------------------------------

def main():
    # Prepare output path
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_path = OUTPUT_DIR / f"run_{ts}{VIDEO_EXT}"

    # Init camera (RGB888 as in your script)
    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
    )
    cam.configure(cfg)
    cam.start()
    time.sleep(0.2)

    writer = None
    fourcc = cv2.VideoWriter_fourcc(*FOURCC)

    print("Recording... press 'q' to stop." if SHOW_VIEW else "Recording... Ctrl+C to stop.")
    prev = time.time()

    try:
        while True:
            # Capture frame (RGB) and convert to BGR for OpenCV writer
            frame_rgb = cam.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            # Lazy-create writer after first frame (ensures exact size)
            if writer is None:
                target_size = (frame_bgr.shape[1], frame_bgr.shape[0])
                writer = cv2.VideoWriter(str(out_path), fourcc, FPS, target_size)
                if not writer.isOpened():
                    # Fallbacks if mp4v fails on your system
                    print("[info] mp4v failed; falling back to XVID/AVI, then MJPG/AVI.")
                    out_path = out_path.with_suffix(".avi")
                    for fallback in ("XVID", "MJPG"):
                        fourcc_fb = cv2.VideoWriter_fourcc(*fallback)
                        writer = cv2.VideoWriter(str(out_path), fourcc_fb, FPS, target_size)
                        if writer.isOpened():
                            break
                    if not writer.isOpened():
                        raise RuntimeError("Could not create video file with any codec")

                print(f"Saving to: {out_path}")
                print(f"Resolution: {target_size[0]}x{target_size[1]} @ {FPS} fps")

            # Write frame
            writer.write(frame_bgr)

            # Optional preview window
            if SHOW_VIEW:
                cv2.imshow("Preview (press q to quit)", frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Simple pacing to approximate target FPS
            now = time.time()
            dt = now - prev
            prev = now
            delay = max(0.0, (1.0 / FPS) - dt)
            if delay > 0:
                time.sleep(delay)

    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        cam.stop()
        cv2.destroyAllWindows()
        print(f"Saved: {out_path}")

if __name__ == "__main__":
    main()