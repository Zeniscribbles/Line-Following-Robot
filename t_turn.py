import time
# from esp32_trx import TRX

# ================= CONFIGURATION =================
TURN_SPEED = 0.5
BASE_SPEED = 0.5

# How stable things must be (tune if needed)
SAMPLE_DT = 0.005
CLEAR_SAMPLES = 8          # how many consecutive "mostly white" reads to confirm cleared
ACQUIRE_SAMPLES = 6        # how many consecutive reads to confirm line acquired & centered
CENTER_ERR_THRESH = 0.25   # abs(line_error) must be below this to stop spinning

# Start-bar (horizontal black marker) clearing
BAR_THRESH = 6             # bar = >= this many sensors see black
BAR_HITS = 3               # debounce hits for confirmed bar detect
BAR_BLIND_S = 0.14         # blind-drive time to shove past bar
BAR_CLEAR_SAMPLES = 4      # consecutive samples below BAR_THRESH to confirm bar cleared
# =================================================

# Initialize TRX globally so we can log from anywhere
# trx = TRX(printDebug=False, blinkDebug=False)
# trx.addPeer(bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]))


def _stable_condition(cond_fn, required_hits, timeout_s):
    """True if cond_fn() is true for required_hits consecutive samples before timeout."""
    start = time.monotonic()
    hits = 0
    while True:
        if time.monotonic() - start > timeout_s:
            return False
        if cond_fn():
            hits += 1
            if hits >= required_hits:
                return True
        else:
            hits = 0
        time.sleep(SAMPLE_DT)


def clear_start_bar(motors, sensors, speed=BASE_SPEED):
    """
    Blind-drive off the horizontal start bar so the turn logic doesn't get confused by it.
    """
    # Optional: reset the internal debounce counter (only matters if you use confirmed)
    if hasattr(sensors, "reset_bar_detector"):
        sensors.reset_bar_detector()

    # If we're not actually on the bar, do nothing.
    on_bar = False
    if hasattr(sensors, "is_black_bar_confirmed"):
        on_bar = sensors.is_black_bar_confirmed(threshold=BAR_THRESH, required_hits=BAR_HITS)
    elif hasattr(sensors, "black_count"):
        on_bar = sensors.black_count() >= BAR_THRESH

    if not on_bar:
        return

    # trx.sendMSG("Start bar: blind drive")
    motors.set_speeds(speed, speed)

    # 1) guaranteed shove forward (no sensor logic)
    time.sleep(BAR_BLIND_S)

    # 2) then wait until we're no longer on a "big blob" of black
    if hasattr(sensors, "black_count"):
        _stable_condition(
            lambda: sensors.black_count() < BAR_THRESH,
            required_hits=BAR_CLEAR_SAMPLES,
            timeout_s=0.8
        )

    motors.stop()
    time.sleep(0.05)


def execute_t_turn(motors, sensors, turn_left=True):
    """
    Executes a 90-degree turn using sensors to define start/stop points.

    Phases:
    0) Blind-drive off the horizontal start bar (if present)
    1) Drive forward until the intersection is cleared (mostly white, stable)
    2) Spin until we're no longer seeing the old bar (mostly white, stable)
    3) Continue spinning until we reacquire the new line stably and roughly centered
    """
    direction_str = "LEFT" if turn_left else "RIGHT"
    # trx.sendMSG(f"Exec T-Turn: {direction_str}")
    print(f"Exec T-Turn: {direction_str}")

    def mostly_white():
        vals = sensors.read_calibrated()
        # "mostly white" means we've moved off the bar/cluster
        return sum(vals) <= 1

    def centered_on_line():
        vals = sensors.read_calibrated()
        black = sum(vals)

        center_ok = (vals[3] == 1 or vals[4] == 1)
        err = sensors.get_line_error()

        # must look like a single line, not a fat intersection blob
        normal_line = (1 <= black <= 4)

        return center_ok and normal_line and (abs(err) < CENTER_ERR_THRESH)

    try:
        # --- PHASE 0: CLEAR THE START BAR (IF PRESENT) ---
        clear_start_bar(motors, sensors, speed=BASE_SPEED)

        # --- PHASE 1: CLEAR THE INTERSECTION ---
        motors.set_speeds(BASE_SPEED, BASE_SPEED)
        ok = _stable_condition(mostly_white, required_hits=CLEAR_SAMPLES, timeout_s=2.0)
        # trx.sendMSG("Intersection Cleared" if ok else "WARN: Align Timeout")

        motors.stop()
        time.sleep(0.08)

        # --- PHASE 2: START SPIN ---
        # trx.sendMSG("Spinning...")
        if turn_left:
            motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            motors.set_speeds(TURN_SPEED, -TURN_SPEED)

        # First: ensure we rotated away from the edge we just left
        _stable_condition(mostly_white, required_hits=6, timeout_s=1.0)

        # --- PHASE 3: ACQUIRE NEW LINE STABLY + CENTERED ---
        ok = _stable_condition(centered_on_line, required_hits=ACQUIRE_SAMPLES, timeout_s=3.0)
        # trx.sendMSG("Line Acquired" if ok else "ERR: Spin Timeout")

    finally:
        motors.stop()
        time.sleep(0.08)
        # trx.sendMSG("Turn Complete")
        print("Turn Complete")
