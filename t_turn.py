import time
import board
import digitalio

from motor_driver_subsystem_2channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID
from esp32_trx import TRX


# ================= CONFIGURATION =================
TURN_SPEED          = 0.4   # Speed for hard 90-degree turns
BASE_SPEED          = 0.35  # Normal driving speed
CORNER_SENSITIVITY  = 0.8   # 0.0 to 1.0 (How dark edge must be to trigger hard turn)
ALL_WHITE_THRESHOLD = 0.2   # If all sensors are below this, we are lost
MEMORY_THRESHOLD    = 0.5   # Only update memory if signal is stronger than this
HARD_TURN_DURATION  = 0.3   # Seconds to lock into a hard turn (blind)
SEND_TRX            = True
# =================================================


# ------------------ TRX HELPERS ------------------
def setup_trx():
    trx = TRX(printDebug=False, blinkDebug=False)
    trx.addPeer(bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]))
    return trx

def log(trx, msg):
    if SEND_TRX and trx:
        trx.sendMSG(msg)


# ---------------- BUTTON / START -----------------
def setup_start_button(pin=board.D12):
    """Returns (has_button, button_or_none)."""
    try:
        btn = digitalio.DigitalInOut(pin)
        btn.direction = digitalio.Direction.INPUT
        btn.pull = digitalio.Pull.UP
        return True, btn
    except Exception:
        return False, None

def wait_for_start(trx, has_button, button):
    log(trx, "Place robot on line. Press Button to Start.")
    if has_button:
        # wait for press (active-low)
        while button.value:
            time.sleep(0.1)
        # wait for release
        while not button.value:
            time.sleep(0.1)
    else:
        time.sleep(2)
    log(trx, "GO!")


# ---------------- CALIBRATION --------------------
def calibrate_sensors(trx, motors, sensors, spin_time=2.0, settle_time=2.0):
    log(trx, "---------------------------------------------")
    log(trx, "CALIBRATION MODE")
    log(trx, "Place robot ON the line.")
    log(trx, f"You have {settle_time} seconds to get ready...")
    time.sleep(settle_time)

    log(trx, "Calibrating... (Spinning)")
    motors.set_speeds(0.3, -0.3)

    start = time.monotonic()
    while (time.monotonic() - start) < spin_time:
        sensors.read_calibrated()
        time.sleep(0.01)

    motors.stop()
    log(trx, "Calibration Done.")
    log(trx, f"Ranges: {[(int(mn), int(mx)) for mn, mx in zip(sensors.min_vals, sensors.max_vals)]}")
    log(trx, "---------------------------------------------")


# ---------------- PAUSE / RESUME -----------------
def handle_pause_if_pressed(trx, has_button, button, motors, pid):
    """
    Pause if button pressed (active-low).
    Returns True if pause/resume happened (caller should reset timing).
    """
    if not has_button:
        return False

    if not button.value:
        motors.stop()
        while not button.value:
            time.sleep(0.1)

        log(trx, "PAUSED. Press button to resume.")

        while button.value:      # wait for press
            time.sleep(0.1)
        while not button.value:  # wait for release
            time.sleep(0.1)

        log(trx, "RESUMING...")
        pid.reset()
        return True

    return False


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
        log(trx, ">>> Hard Left Lock!")
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        return True, now + HARD_TURN_DURATION, -1

    if vals[7] > CORNER_SENSITIVITY:
        log(trx, ">>> Hard Right Lock!")
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)
        return True, now + HARD_TURN_DURATION, 1

    return False, 0.0, 0

def handle_lost_line(trx, motors, last_valid_error):
    if last_valid_error < 0:
        log(trx, "Lost! Spinning Left...")
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
    else:
        log(trx, "Lost! Spinning Right...")
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)

def pid_drive(motors, sensors, pid, dt):
    err = sensors.get_line_error()
    correction = pid.update(0.0, err, dt)
    motors.set_speeds(BASE_SPEED - correction, BASE_SPEED + correction)


# ------------------ MAIN LOOP --------------------
def run_line_follower(trx, motors, sensors, pid, has_button, button):
    last_valid_error = 0.0
    last_time = time.monotonic()

    turn_lock_until = 0.0
    turn_lock_direction = 0  # -1 left, +1 right

    while True:
        now = time.monotonic()

        paused = handle_pause_if_pressed(trx, has_button, button, motors, pid)
        if paused:
            last_time = time.monotonic()
            continue

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
    use_button_pin=board.D12,
    kp=0.40, ki=0.01, kd=0.055,
    do_calibration=True,
):
    """
    Call this from another file to run the T-turn behavior.

    Example:
        from t_turns_module import run_t_turns
        run_t_turns()
    """
    trx = TRX(printDebug=False, blinkDebug=False)
    trx.addPeer(peer_mac)

    log(trx, "Initializing Reactive Turn Test...")

    motors = MotorDriver()
    sensors = ReflectiveArray()

    pid = PID(kp=kp, ki=ki, kd=kd)

    has_button, button = setup_start_button(use_button_pin)

    if do_calibration:
        calibrate_sensors(trx, motors, sensors)

    wait_for_start(trx, has_button, button)

    try:
        run_line_follower(trx, motors, sensors, pid, has_button, button)
    except KeyboardInterrupt:
        motors.stop()
