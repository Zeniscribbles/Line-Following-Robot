import time
import random

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
    # These timings are very track-dependent. Keep them close to your original values.
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
    # Goal: avoid returning while still on fork/intersection junk (multiple lines / blobs).
    # Heuristic for digital sensors:
    #   - center sees black
    #   - black_count is in a "normal line" range (not 0, not huge)
    # Tune BLACK_MIN/MAX for your track.
    BLACK_MIN = 1
    BLACK_MAX = 4
    STABLE_HITS_REQUIRED = 6

    start = time.monotonic()
    stable_hits = 0

    while True:
        # Safety timeout so we never get stuck here
        if time.monotonic() - start > 2.0:
            print("WARN: Fork reacquire timeout, returning anyway.")
            break

        vals = sensors.read_calibrated()
        black = sum(vals)
        center_ok = (vals[3] == 1 or vals[4] == 1)

        good = center_ok and (BLACK_MIN <= black <= BLACK_MAX)

        if good:
            stable_hits += 1
            if stable_hits >= STABLE_HITS_REQUIRED:
                break
        else:
            stable_hits = 0

        # Gentle creep forward while searching for the “real” line
        motors.set_speeds(0.45, 0.45)
        time.sleep(0.01)

    motors.stop()
    time.sleep(0.05)
    print("Fork entry complete. Resuming Line Follow.")
