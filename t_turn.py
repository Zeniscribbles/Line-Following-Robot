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
SEND_TRX            = True
# =================================================


# --------------- SENSOR STATE --------------------
def read_line_state(sensors, last_valid_error):
    vals = sensors.read_calibrated()
    max_reflection = max(vals)
    is_lost = (max_reflection < ALL_WHITE_THRESHOLD)

    if max_reflection > MEMORY_THRESHOLD:
        last_valid_error = sensors.get_line_error()

    return vals, max_reflection, is_lost, last_valid_error


# -------------- CONTROL LOGIC --------------------
def apply_turn_lock(motors, now, turn_lock_until, turn_lock_direction):
    if now < turn_lock_until:
        if turn_lock_direction == -1:
            motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            motors.set_speeds(TURN_SPEED, -TURN_SPEED)
        return True
    return False

def trigger_hard_turn_if_needed(trx, motors, vals, now):
    if vals[0] > CORNER_SENSITIVITY:
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        return True, now + HARD_TURN_DURATION, -1

    if vals[7] > CORNER_SENSITIVITY:
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)
        return True, now + HARD_TURN_DURATION, 1

    return False, 0.0, 0

def handle_lost_line(trx, motors, last_valid_error):
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

        vals, max_reflection, is_lost, last_valid_error = read_line_state(sensors, last_valid_error)

        if apply_turn_lock(motors, now, turn_lock_until, turn_lock_direction):
            continue

        triggered, new_until, new_dir = trigger_hard_turn_if_needed(trx, motors, vals, now)
        if triggered:
            turn_lock_until = new_until
            turn_lock_direction = new_dir
            continue

        if is_lost:
            handle_lost_line(trx, motors, last_valid_error)
            continue

        pid_drive(motors, sensors, pid, dt)


# ================== PUBLIC ENTRYPOINT ==================
def run_t_turns(
    *,
    peer_mac=bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]),
    kp=0.40, ki=0.01, kd=0.055,
    do_calibration=True,
):
    """
    Call this from another file to run the T-turn behavior.

    Example:
        from t_turns_module import run_t_turns
        run_t_turns()
    """
    motors = MotorDriver()
    sensors = ReflectiveArray()
    pid = PID(kp=kp, ki=ki, kd=kd)


    try:
        run_line_follower(motors, sensors, pid)
    except KeyboardInterrupt:
        motors.stop()
