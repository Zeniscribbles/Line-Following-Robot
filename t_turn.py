import time, random

TURN_SPEED          = 0.40
CORNER_SENSITIVITY  = 0.80
ALL_WHITE_THRESHOLD = 0.20
MEMORY_THRESHOLD    = 0.50
HARD_TURN_DURATION  = 0.30

MAX_TURN_TIME   = 1.8
CENTER_TOL      = 0.10
STABLE_SAMPLES  = 4

def execute_t_turn(motors, sensors, turn_left=None, **kwargs):
    """
    If turn_left is None:
      - choose the available branch (left vs right) using sensors
      - if both look available, pick randomly
    """

    # Read once to decide direction
    vals = sensors.read_calibrated()

    left_seen  = vals[0]  > CORNER_SENSITIVITY
    right_seen = vals[-1] > CORNER_SENSITIVITY

    if turn_left is None:
        if left_seen and not right_seen:
            turn_left = True
        elif right_seen and not left_seen:
            turn_left = False
        else:
            # both or neither are obvious → pick randomly
            turn_left = random.choice([True, False])

    direction = -1 if turn_left else 1  # -1 left, +1 right
    last_valid_error = 0.0

    def spin(dir_sign):
        if dir_sign < 0:
            motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            motors.set_speeds(TURN_SPEED, -TURN_SPEED)

    # Phase 0: hard lock (blind commit)
    lock_until = time.monotonic() + HARD_TURN_DURATION
    while time.monotonic() < lock_until:
        spin(direction)
        time.sleep(0.005)

    # Phase 1: seek + recover until centered
    start = time.monotonic()
    stable = 0

    while (time.monotonic() - start) < MAX_TURN_TIME:
        vals = sensors.read_calibrated()
        max_ref = max(vals)
        lost = max_ref < ALL_WHITE_THRESHOLD

        if max_ref > MEMORY_THRESHOLD:
            last_valid_error = sensors.get_line_error()

        if not lost:
            err = sensors.get_line_error()
            if abs(err) < CENTER_TOL:
                stable += 1
                if stable >= STABLE_SAMPLES:
                    break
            else:
                stable = 0

        if lost:
            # recovery direction based on last error
            spin(-1 if last_valid_error < 0 else 1)
        else:
            # keep turning the chosen way until centered
            spin(direction)

        time.sleep(0.005)

    motors.stop()
    time.sleep(0.05)
