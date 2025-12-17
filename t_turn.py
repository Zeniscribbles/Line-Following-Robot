import time
import board
import digitalio

from motor_2_channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID


# ================= CONFIGURATION =================
TURN_SPEED          = 0.4
BASE_SPEED          = 0.35
CORNER_SENSITIVITY  = 0.8
ALL_WHITE_THRESHOLD = 0.2
MEMORY_THRESHOLD    = 0.5
HARD_TURN_DURATION  = 0.3
BAR_THRESH          = 0.80     # Matched to robot_main
BAR_COUNT_THRESH    = 6        # Matched to robot_main
# =================================================


# --------------- SENSOR STATE --------------------
def read_line_state(sensors, last_valid_error):
    vals = sensors.read_calibrated()
    max_reflection = max(vals)
    is_lost = (max_reflection < ALL_WHITE_THRESHOLD)

    if max_reflection > MEMORY_THRESHOLD:
        last_valid_error = sensors.get_line_error()

    return vals, max_reflection, is_lost, last_valid_error

def is_black_bar(vals):
    """Detects if we have hit the next black bar (Exit Condition)."""
    count = sum(1 for v in vals if v >= BAR_THRESH)
    return count >= BAR_COUNT_THRESH

# -------------- CONTROL LOGIC --------------------
def apply_turn_lock(motors, now, turn_lock_until, turn_lock_direction):
    if now < turn_lock_until:
        if turn_lock_direction == -1:
            motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            motors.set_speeds(TURN_SPEED, -TURN_SPEED)
        return True
    return False

def trigger_hard_turn_if_needed(motors, vals, now):
    if vals[0] > CORNER_SENSITIVITY:
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        return True, now + HARD_TURN_DURATION, -1

    if vals[7] > CORNER_SENSITIVITY:
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)
        return True, now + HARD_TURN_DURATION, 1

    return False, 0.0, 0

def handle_lost_line(motors, last_valid_error):
    if last_valid_error < 0:
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
    else:
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)

def pid_drive(motors, sensors, pid, dt):
    err = sensors.get_line_error()
    correction = pid.update(0.0, err, dt)
    motors.set_speeds(BASE_SPEED - correction, BASE_SPEED + correction)


# ------------------ MAIN LOOP --------------------
def run_line_follower(motors, sensors, pid):
    last_valid_error = 0.0
    last_time = time.monotonic()

    turn_lock_until = 0.0
    turn_lock_direction = 0  # -1 left, +1 right


    while True:
            now = time.monotonic()

            dt = now - last_time
            last_time = now

            # 1. Read Sensors
            vals, max_reflection, is_lost, last_valid_error = read_line_state(sensors, last_valid_error)

            # 2. EXIT CONDITION: Check for next state trigger
            if is_black_bar(vals):
                print(">>> T-TURN COMPLETE: Bar detected. Returning to Main.")
                motors.stop()
                return # Breaks the loop and returns control to robot_main

            # 3. Turn Lock
            if apply_turn_lock(motors, now, turn_lock_until, turn_lock_direction):
                continue

            # 4. Check for Intersections
            triggered, new_until, new_dir = trigger_hard_turn_if_needed(motors, vals, now)
            if triggered:
                turn_lock_until = new_until
                turn_lock_direction = new_dir
                continue

            # 5. Lost Line Logic
            if is_lost:
                handle_lost_line(motors, last_valid_error)
                continue

            # 6. Standard PID
            pid_drive(motors, sensors, pid, dt)

# ================== PUBLIC ENTRYPOINT ==================
def run_t_turns(motors, sensors, **kwargs):
    """
    Revised entrypoint to accept existing hardware objects.
    """
    # Extract config from kwargs or use defaults
    kp = kwargs.get('kp', 0.40)
    ki = kwargs.get('ki', 0.01)
    kd = kwargs.get('kd', 0.055)

    # Create a local PID instance (safe to do)
    pid = PID(kp=kp, ki=ki, kd=kd)

    try:
        run_line_follower(motors, sensors, pid)
    except KeyboardInterrupt:
        motors.stop()
        raise # Allow Ctrl+C to propagate up
