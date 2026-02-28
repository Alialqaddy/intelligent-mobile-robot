# motor_check.py
from motor_control import forward, backward, spin_left, spin_right, stop_all
import time

def pause(msg="Press Enter to continue..."):
    input(msg)

try:
    print("Lift the wheels off the ground before testing.")

    pause("Step 1: Move forward...")
    forward(0.5); time.sleep(1.5); stop_all()

    pause("Step 2: Move backward...")
    backward(0.5); time.sleep(1.5); stop_all()

    pause("Step 3: Spin left in place...")
    spin_left(0.6); time.sleep(1.2); stop_all()

    pause("Step 4: Spin right in place...")
    spin_right(0.6); time.sleep(1.2); stop_all()

    pause("Short forward pulse test...")
    for _ in range(4):
        forward(0.4); time.sleep(0.3)
        stop_all(); time.sleep(0.3)

    print("Test finished.")
finally:
    # Ensure everything is turned off in case of error/exit
    stop_all()