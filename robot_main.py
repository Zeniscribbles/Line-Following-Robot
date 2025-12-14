import time
import board
from motor_driver_subsystem_2channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID
from esp32_trx import TRX

# Import the logic from other files
import straight_away
import serpentine
import fork
import t_turn

# --- CONFIGURATION ---
TRACK_MAP = [
    'STRAIGHT',    # Start line to T-Turn
    'TTURN_LEFT',  # The T-Turn
    'STRAIGHT',    # T-Turn to Fork
    'FORK',        # The Fork
    'SERPENTINE',  # Inside the fork path
    'STRAIGHT',    # Exit of fork/serpentine to finish
    'FINISH'       # Stop
]

# Initialize TRX globally so we can log from anywhere
trx = TRX(printDebug=False, blinkDebug=False)
trx.addPeer(bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]))

def execute_u_turn(motors, sensors, turn_left=True):
    """
    Executes ~180-degree turn and reacquires the line going the opposite direction.

    Strategy:
    - Spin in place.
    - Don't allow reacquire until either:
        a) we've seen mostly-white (line gone) at least once, and
        b) a minimum spin time has elapsed
    - Then require stable centered reacquire.
    """
    direction_str = "LEFT" if turn_left else "RIGHT"
    trx.sendMSG(f"Exec U-Turn: {direction_str}")
    print(f"Exec U-Turn: {direction_str}")

    SAMPLE_DT = 0.005
    MIN_SPIN_TIME = 0.60          # tune 0.5–0.9 depending on your chassis
    MAX_SPIN_TIME = 4.0
    WHITE_THRESH = 1              # sum(vals) <= this == "mostly white"
    ACQUIRE_SAMPLES = 8
    CENTER_ERR_THRESH = 0.25      # loosen to 0.35 if it never locks

    def mostly_white():
        vals = sensors.read_calibrated()
        return sum(vals) <= WHITE_THRESH

    def centered_on_line():
        vals = sensors.read_calibrated()
        black = sum(vals)
        center_ok = (vals[3] == 1 or vals[4] == 1)
        err = sensors.get_line_error()
        normal_line = (1 <= black <= 4)
        return center_ok and normal_line and (abs(err) < CENTER_ERR_THRESH)

    motors.stop()
    time.sleep(0.05)

    # Start spinning
    if turn_left:
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
    else:
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)

    spin_start = time.monotonic()
    saw_white = False
    stable = 0

    while True:
        elapsed = time.monotonic() - spin_start
        if elapsed > MAX_SPIN_TIME:
            trx.sendMSG("ERR: U-Turn Timeout")
            break

        if mostly_white():
            saw_white = True

        # gate reacquire so we don't "micro-turn"
        if elapsed < MIN_SPIN_TIME or not saw_white:
            time.sleep(SAMPLE_DT)
            continue

        if centered_on_line():
            stable += 1
            if stable >= ACQUIRE_SAMPLES:
                trx.sendMSG("U-Turn Line Acquired")
                break
        else:
            stable = 0

        time.sleep(SAMPLE_DT)

    motors.stop()
    time.sleep(0.08)
    trx.sendMSG("U-Turn Complete")
    print("U-Turn Complete")


def run_robot():
    # 1. Initialize Hardware
    motors = MotorDriver()
    sensors = ReflectiveArray()

    # Default PID (will be updated by states)
    pid = PID(kp=0.8, ki=0.0, kd=0.1)

    current_state_idx = 0
    current_speed = 0.5

    # --- Control Safeties / Tunables ---
    MAX_DT = 0.05            # cap dt to reduce derivative weirdness from jitter
    MAX_CORRECTION = 0.45    # cap correction to avoid violent snap turns
    BAR_HITS_REQUIRED = 3    # consecutive reads required to confirm black bar

    # ================= CALIBRATION BLOCK =================
    print("---------------------------------------------")
    print("CALIBRATION MODE")
    print("Place robot ON the line.")
    print("You have 2 seconds to get ready...")
    time.sleep(2)

    print("Calibrating... (Spinning)")
    motors.set_speeds(0.3, -0.3)

    cal_start = time.monotonic()
    while time.monotonic() - cal_start < 2.0:
        sensors.read_calibrated()  # NOTE: your current sensor code doesn't update min/max
        time.sleep(0.01)

    motors.stop()
    print("Calibration Done.")
    print("Ranges:", [(int(min_v), int(max_v)) for min_v, max_v in zip(sensors.min_vals, sensors.max_vals)])
    print("---------------------------------------------")
    # =====================================================

    print("ROBOT READY. PRESS CTRL+C to STOP.")
    print(f"Current State: {TRACK_MAP[current_state_idx]}")

    last_time = time.monotonic()
    black_bar_hits = 0

    try:
        while True:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            # Cap dt to prevent derivative blowing up from jitter / pauses
            if dt > MAX_DT:
                dt = MAX_DT

            # --- A. Confirmed Black Bar Detection (Consecutive Hits) ---
            if sensors.is_black_bar():
                black_bar_hits += 1
            else:
                black_bar_hits = 0

            if black_bar_hits >= BAR_HITS_REQUIRED:
                black_bar_hits = 0  # reset so we don't double-trigger
                print(">>> BLACK BAR DETECTED (CONFIRMED) <<<")

                motors.stop()
                time.sleep(0.05)

                # Move to next state
                current_state_idx += 1
                if current_state_idx >= len(TRACK_MAP):
                    print("Track Complete!")
                    break

                new_state = TRACK_MAP[current_state_idx]
                print(f"Transitioning to: {new_state}")

                # --- B. Handle One-Time Entry Logic ---
                if new_state == 'FORK':
                    # If you later update fork.handle_fork to accept sensors,
                    # change this to: fork.handle_fork(motors, sensors, direction="random")
                    fork.handle_fork(motors, sensors, direction="random")

                elif new_state == 'TTURN_LEFT':
                    t_turn.execute_t_turn(motors, sensors, turn_left=True)

                elif new_state == 'TTURN_RIGHT':
                    t_turn.execute_t_turn(motors, sensors, turn_left=False)

                elif new_state == 'FINISH':
                    break

                # Drive forward to clear the bar AND wait for it to disappear
                motors.set_speeds(0.4, 0.4)
                clear_start = time.monotonic()
                while sensors.is_black_bar() and (time.monotonic() - clear_start < 1.0):
                    time.sleep(0.01)
                time.sleep(0.05)

                # --- C. Update PID Params for the new section ---
                if new_state == 'SERPENTINE':
                    params = serpentine.get_params()
                else:
                    params = straight_away.get_params()

                pid.kp = params.get('kp', pid.kp)
                pid.ki = params.get('ki', pid.ki)
                pid.kd = params.get('kd', pid.kd)
                current_speed = params.get('speed', current_speed)

                # Critical: reset PID/timing after any blocking maneuver
                pid.reset()
                last_time = time.monotonic()

                continue

            # --- D. Normal Line Following (PID) ---
            line_error = sensors.get_line_error()

            # NOTE: if steering seems reversed, try:
            # correction = pid.update(0.0, -line_error, dt)
            correction = pid.update(0.0, line_error, dt)

            # Optional but very helpful: cap correction so straightaways don't snap-turn
            if correction > MAX_CORRECTION:
                correction = MAX_CORRECTION
            elif correction < -MAX_CORRECTION:
                correction = -MAX_CORRECTION

            left_speed = current_speed - correction
            right_speed = current_speed + correction

            motors.set_speeds(left_speed, right_speed)
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nCTRL+C detected. Stopping...")

    finally:
        motors.stop()
        print("Mission Accomplished.")

if __name__ == "__main__":
    run_robot()
