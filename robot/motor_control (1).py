import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from config import IN1, IN2, IN3, IN4, ENA, ENB, SWAP_SIDES, TURN_SPEED 

# Motor initialization
L_FWD = DigitalOutputDevice(IN1)
L_BWD = DigitalOutputDevice(IN2)
R_FWD = DigitalOutputDevice(IN3)
R_BWD = DigitalOutputDevice(IN4)

ENA_PWM = PWMOutputDevice(ENA, frequency=1000, initial_value=0.0)
ENB_PWM = PWMOutputDevice(ENB, frequency=1000, initial_value=0.0)

# ====== PWM helper ======
def _set_pwm(v: float):
    v = max(0.0, min(1.0, v))
    ENA_PWM.value = v
    ENB_PWM.value = v

def stop_all():
    L_FWD.off(); L_BWD.off()
    R_FWD.off(); R_BWD.off()
    _set_pwm(0.0)
    time.sleep(0.02)

# ====== Correct directions ======
def forward(speed: float):
    _set_pwm(speed)
    # Both wheels forward
    L_BWD.off(); L_FWD.on()
    R_BWD.off(); R_FWD.on()

def backward(speed: float):
    _set_pwm(speed)
    # Both wheels backward
    L_FWD.off(); L_BWD.on()
    R_FWD.off(); R_BWD.on()

def spin_right(speed: float = TURN_SPEED):
    _set_pwm(speed)
    if SWAP_SIDES:
        L_BWD.on(); L_FWD.off()
        R_FWD.on(); R_BWD.off()
    else:
        L_BWD.off(); L_FWD.on()
        R_BWD.off(); R_FWD.on()

def spin_left(speed: float = TURN_SPEED):
    _set_pwm(speed)
    if SWAP_SIDES:
        L_FWD.on(); L_BWD.off()
        R_BWD.on(); R_FWD.off()
    else:
        L_FWD.off(); L_BWD.on()
        R_FWD.off(); R_FWD.on()


# --- helpers ---
def _clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))

def drive_forward(left_speed: float, right_speed: float):
    """Drive forward while allowing different speeds per side (0..1)."""
    left_speed  = _clamp(left_speed)
    right_speed = _clamp(right_speed)

    # direction: forward
    L_BWD.off(); R_BWD.off()
    L_FWD.on();  R_FWD.on()

    ENA_PWM.value = left_speed   # left motor enable (ENA)
    ENB_PWM.value = right_speed  # right motor enable (ENB)

def forward_lr(left_pwm: float, right_pwm: float):
    # clamp
    left_pwm = max(0.0, min(1.0, left_pwm))
    right_pwm = max(0.0, min(1.0, right_pwm))
    # forward direction
    L_BWD.off(); R_BWD.off()
    L_FWD.on();  R_FWD.on()
    # independent speeds
    ENA_PWM.value = right_pwm
    ENB_PWM.value = left_pwm