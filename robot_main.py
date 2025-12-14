import time
import board
import digitalio
from motor_driver_subsystem_2channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID
from esp32_trx import TRX

import straight_away
import serpentine
import fork
import t_turn

# ---------------- CONFIG ----------------
MAX_DT = 0.05
MAX_CORRECTION = 0.45
BAR_HITS_REQUIRED = 3
# ----------------------------------------

trx = TRX(printDebug=False, blinkDebug=False)
trx.addPeer(bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]))


def execute_u_turn(motors, sensors):
    """
    Executes ~180-degree turn and reacquires the line going the opposite direction.
    NO hardcoded left/right in main: we auto-pick a spin direction based on sensor snapshot.
    """

    # Auto-pick direction from current sensor distribution (simple + effective)
    vals0 = sensors.read_calibrated()
    left_black = sum(vals0[:4])
    right_black = sum(vals0[4:])
    turn_left = (left_black >= right_black)  # tie -> left, but sensor-driven (not a constant)

    direction_str = "LEFT" if turn_left else "RIGHT"
    trx.sendMSG(f"Exec U-Turn: AUTO -> {direction_str}")
    print(f"Exec U-Turn: AUTO -> {direction_str}")

    SAMPLE_DT = 0.005
    MIN_SPIN_TIME = 0.60
    MAX_SPIN_TIME = 4.0
    WHITE_THRESH = 1
    ACQUIRE_SAMPLES = 8
    CENTER_ERR_THRESH = 0.25

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

    TURN_SPEED = 0.5
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


def event_for_bar(bar_num, forward):
    """
    5 bars per pass.

    Forward:  1=Serpentine, 2=Straight, 3=T-turn, 4=Fork, 5=U-turn
    Reverse:  1=Fork,      2=T-turn,    3=Straight, 4=Serpentine, 5=U-turn
    """
    if forward:
        return {1: "SERPENTINE", 2: "STRAIGHT", 3: "TTURN", 4: "FORK", 5: "UTURN"}[bar_num]
    else:
        return {1: "FORK", 2: "TTURN", 3: "STRAIGHT", 4: "SERPENTINE", 5: "UTURN"}[bar_num]


def set_follow_params(pid, section_name):
    if section_name == "SERPENTINE":
        params = serpentine.get_params()
    else:
        params = straight_away.get_params()

    pid.kp = params.get("kp", pid.kp)
    pid.ki = params.get("ki", pid.ki)
    pid.kd = params.get("kd", pid.kd)
    return params.get("speed", 0.5)


def clear_black_bar(motors, sensors):
    motors.set_speeds(0.4, 0.4)
    clear_start = time.monotonic()
    while sensors.is_black_bar() and (time.monotonic() - clear_start < 1.0):
        time.sleep(0.01)
    time.sleep(0.05)


def run_robot():
    motors = MotorDriver()
    sensors = ReflectiveArray()

    # --- BUTTON SETUP ---
    try:
        start_button = digitalio.DigitalInOut(board.D12)
        start_button.direction = digitalio.Direction.INPUT
        start_button.pull = digitalio.Pull.UP
        HAS_BUTTON = True
        print("Button initialized on D12")
    except Exception as e:
        print(f"Button setup skipped: {e}")
        HAS_BUTTON = False
    # --------------------

    pid = PID(kp=0.8, ki=0.0, kd=0.1)

    current_speed = set_follow_params(pid, "STRAIGHT")

    forward = True
    bar_num = 0
    black_bar_hits = 0

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
        sensors.read_calibrated()
        time.sleep(0.01)

    motors.stop()
    print("Calibration Done.")
    print("Ranges:", [(int(min_v), int(max_v)) for min_v, max_v in zip(sensors.min_vals, sensors.max_vals)])
    print("---------------------------------------------")
    # =====================================================

    # --- WAIT FOR START BUTTON ---
    if HAS_BUTTON:
        print(">>> HOLD BUTTON to START... <<<")
        while start_button.value:
            time.sleep(0.1)
        while not start_button.value:
            time.sleep(0.1)
        print("GO!")
    else:
        print("No button detected. Starting in 2 seconds...")
        time.sleep(2)
    # -----------------------------

    last_time = time.monotonic()

    try:
        while True:
            # --- STOP BUTTON CHECK ---
            if HAS_BUTTON and not start_button.value:
                print(">>> STOP BUTTON PRESSED")
                motors.stop()
                while not start_button.value:
                    time.sleep(0.1)
                break
            # ------------------------

            now = time.monotonic()
            dt = now - last_time
            last_time = now
            if dt > MAX_DT:
                dt = MAX_DT

            # --- Black bar debounce ---
            if sensors.is_black_bar():
                black_bar_hits += 1
            else:
                black_bar_hits = 0

            if black_bar_hits >= BAR_HITS_REQUIRED:
                black_bar_hits = 0
                motors.stop()
                time.sleep(0.05)

                bar_num = (bar_num % 5) + 1
                event = event_for_bar(bar_num, forward)
                dir_str = "FWD" if forward else "REV"
                msg = f"BAR {bar_num}/5 ({dir_str}) -> {event}"
                print(">>> " + msg)
                trx.sendMSG(msg)

                # ---- Execute event ----
                if event == "SERPENTINE":
                    current_speed = set_follow_params(pid, "SERPENTINE")

                elif event == "STRAIGHT":
                    current_speed = set_follow_params(pid, "STRAIGHT")

                elif event == "TTURN":
                    # No hardcoded direction here: let your t_turn module decide its own behavior.
                    t_turn.execute_t_turn(motors, sensors)
                    current_speed = set_follow_params(pid, "STRAIGHT")

                elif event == "FORK":
                    fork.handle_fork(motors, sensors, direction="random")
                    current_speed = set_follow_params(pid, "SERPENTINE" if forward else "STRAIGHT")

                elif event == "UTURN":
                    execute_u_turn(motors, sensors)
                    forward = not forward
                    bar_num = 0
                    current_speed = set_follow_params(pid, "STRAIGHT")

                clear_black_bar(motors, sensors)

                pid.reset()
                last_time = time.monotonic()
                continue

            # --- Normal PID line following ---
            line_error = sensors.get_line_error()
            correction = pid.update(0.0, line_error, dt)

            if correction > MAX_CORRECTION:
                correction = MAX_CORRECTION
            elif correction < -MAX_CORRECTION:
                correction = -MAX_CORRECTION

            motors.set_speeds(current_speed - correction, current_speed + correction)
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nCTRL+C detected. Stopping...")

    finally:
        motors.stop()
        print("Mission Accomplished.")


if __name__ == "__main__":
    run_robot()
