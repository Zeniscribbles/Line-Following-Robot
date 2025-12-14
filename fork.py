import time
import random

# Match main/t_turn interpretation of calibrated floats
LINE_THRESH = 0.55  # "this sensor sees line"

def handle_fork(motors, sensors, direction="random"):
    """
    Executes a fork entry maneuver, then waits until a stable single line is reacquired.

    direction: 'left', 'right', 'straight', or 'random'
    """
    print("Entering Fork Logic...")

    if direction == "random":
        choice = random.choice(["left", "right", "straight"])
    else:
        choice = direction

    if choice not in ("left", "right", "straight"):
        raise ValueError("direction must be 'left', 'right', 'straight', or 'random'")

    print(f"Chose: {choice}")

    # ---------------- Phase 1: Commit to a branch (mostly blind) ----------------
    if choice == "left":
        motors.set_speeds(-0.3, 0.3)
        time.sleep(0.20)
        motors.set_speeds(0.4, 0.5)  # arc left
        time.sleep(0.50)

    elif choice == "straight":
        motors.set_speeds(0.6, 0.6)
        time.sleep(0.40)

    else:  # right
        motors.set_speeds(0.3, -0.3)
        time.sleep(0.20)
        motors.set_speeds(0.5, 0.4)  # arc right
        time.sleep(0.50)

    # ---------------- Phase 2: Reacquire a clean line before returning ----------------
    # Same idea as before, but adapted for float sensors:
    # - black_count = number of sensors above LINE_THRESH
    BLACK_MIN = 1
    BLACK_MAX = 4
    STABLE_HITS_REQUIRED = 6

    start = time.monotonic()
    stable_hits = 0

    while True:
        if time.monotonic() - start > 2.0:
            print("WARN: Fork reacquire timeout, returning anyway.")
            break

        vals = sensors.read_calibrated()

        black_count = sum(1 for v in vals if v >= LINE_THRESH)
        center_ok = (vals[3] >= LINE_THRESH) or (vals[4] >= LINE_THRESH)

        good = center_ok and (BLACK_MIN <= black_count <= BLACK_MAX)

        if good:
            stable_hits += 1
            if stable_hits >= STABLE_HITS_REQUIRED:
                break
        else:
            stable_hits = 0

        motors.set_speeds(0.45, 0.45)
        time.sleep(0.01)

    motors.stop()
    time.sleep(0.05)
    print("Fork entry complete. Resuming Line Follow.")
