import time
import board
import digitalio
from motor_2_channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID
import esp32_trx as trx

# Import maneuvers
import fork
import t_turn

# ---------------- WRAPPERS ----------------
def do_fork_action(motors, sensors, **kwargs):
    """Aligns, chooses a path, executes it, and saves the choice."""
    global LAST_FORK_CHOICE
    # DEBUG
    trx.sendMSG(">> ACTION: Executing Fork...")
    fork.standard_align(motors, sensors)
    LAST_FORK_CHOICE = fork.execute_random_fork(motors)
    trx.sendMSG(f">> ACTION RESULT: Chose {LAST_FORK_CHOICE}")


def fork_return_action(motors, sensors, **kwargs):
    """Uses the saved choice to align correctly and cross the bar."""
    global LAST_FORK_CHOICE
    if LAST_FORK_CHOICE is None:
        trx.sendMSG("WARNING: No choice saved. Defaulting to CENTER.")
        LAST_FORK_CHOICE = "CENTER"
    trx.sendMSG(f">> ACTION: Return from {LAST_FORK_CHOICE}")
    fork.force_align_and_cross(motors, sensors, LAST_FORK_CHOICE)

    trx.sendMSG(">> ACTION: Executing 180 Turn...")
    fork.turn_180(motors, sensors)


# ---------------- CONFIGURATION ----------------
# --- TUNING -------
KP = 0.75
KI = 0.01
KD = 0.07
BASE_SPEED = 0.15

# 1. TIMING & SENSITIVITY
MAX_DT = 0.05
MAX_CORRECTION = 0.45
BAR_HITS_REQUIRED = 3

# --- DRIVE TIMES ---
BAR_CLEAR_TIME = 0.6  # Standard time to clear T-Turn/Fork bars
START_CLEAR_TIME = 0.1  # NEW: Tiny blip just to get off the Start Linef

# 2. SENSOR THRESHOLDS
# (Adjusted slightly lower to be safer, based on your previous issues)
BAR_THRESH = 0.60  # Black > 0.60
BAR_COUNT_THRESH = 7  # Bar = 4+ sensors black
GAP_THRESH = 0.10  # Line Lost = All sensors < 0.10

# >>>>>> HARDWARE TRIM <<<<<<
# IF ROBOT PULLS LEFT:  REDUCE RIGHT_TRIM_VAL (e.g. 0.95)
# IF ROBOT PULLS RIGHT: REDUCE LEFT_TRIM_VAL  (e.g. 0.95)
LEFT_TRIM_VAL = 1.0
RIGHT_TRIM_VAL = 0.85

# 3. TRACK SEQUENCE
TRACK_SEQUENCE = [
    {"name": "SERPENTINE",     "action": None,                 "gaps_allowed": False},
    {"name": "STRAIGHTAWAY",   "action": None,                 "gaps_allowed": True},
    {"name": "DO_TTURN",       "action": t_turn.run_t_turns,   "gaps_allowed": False},
    {"name": "DO_FORK", "action": do_fork_action, "gaps_allowed": False},
    {"name": "FORK_RETURN", "action": fork_return_action, "gaps_allowed": False},
    
    {"name": "DO_FORK", "action": do_fork_action, "gaps_allowed": False},
    {"name": "DO_TTURN", "action": t_turn.run_t_turns, "gaps_allowed": False},
    {"name": "STRAIGHTAWAY", "action": None, "gaps_allowed": True},
    {"name": "SERPENTINE", "action": None, "gaps_allowed": False},
    {"name": "END_SERP_RET", "action": None, "gaps_allowed": False},
]

# -----------------------------------------------
# Wifi Enabled Debugging Statements:
trx.setDebug(
    printDebug=True, blinkDebug=True
)  # Set printDebug=True so you see it in Mu
trx.addPeer(bytes([0xEC, 0xDA, 0x3B, 0x61, 0x58, 0x58]))


def current_frame_is_bar(vals):
    # Count how many sensors see black
    black_count = sum(1 for v in vals if v > BAR_THRESH)

    # >>>>>> #2: EDGE GUARD <<<<<<
    # If we see 7 or 8, it's definitely a bar.
    if black_count >= 7:
        return True
    # If we see 6, we only count it IF the "Outer" sensors aren't completely empty.
    # This prevents a tight "inside turn" (sensors 0,1,2,3,4,5) from triggering.
    # At least ONE of the far edges (0 or 7) must be involved for it to be a traverse bar.
    if black_count >= 6:
        if vals[0] > BAR_THRESH or vals[7] > BAR_THRESH:
            return True
    return False


def is_line_lost(vals):
    return max(vals) < GAP_THRESH


def run_robot():
    # --- HARDWARE SETUP ---
    motors = MotorDriver()
    sensors = ReflectiveArray()

    try:
        start_button = digitalio.DigitalInOut(board.D12)
        start_button.direction = digitalio.Direction.INPUT
        start_button.pull = digitalio.Pull.UP
        HAS_BUTTON = True
        trx.sendMSG("DEBUG: Button initialized on D12")
    except Exception as e:
        trx.sendMSG(f"DEBUG: Button setup skipped: {e}")
        HAS_BUTTON = False

    # ==========================================================
    # CALIBRATION LOOP
    # ==========================================================
    trx.sendMSG(">>> RESET")

    calibration_successful = False

    while not calibration_successful:
        trx.sendMSG("--- CALIBRATION REQUIRED ---")
        trx.sendMSG("1. Place robot on White")
        trx.sendMSG("2. Ensure it crosses BLACK line during spin")
        trx.sendMSG(">>> Press Button to Spin...")

        # Wait for button press
        if HAS_BUTTON:
            while start_button.value:
                time.sleep(0.1)
            while not start_button.value:
                time.sleep(0.1)
        else:
            time.sleep(2)
        trx.sendMSG("Spinning...")

        motors.set_speeds(0.25, -0.25)
        start_cal = time.monotonic()
        while time.monotonic() - start_cal < 2.0:
            sensors.read_calibrated()
        motors.stop()

        trx.sendMSG("--- CALIBRATION DONE ---")
        # ----------------------------------------------

        start_cal = time.monotonic()
        while time.monotonic() - start_cal < 2.5:
            sensors.read_calibrated()
        motors.stop()

        # VALIDATE CALIBRATION
        test_read = sensors.read_calibrated()
        max_test = max(test_read)

        trx.sendMSG(f"DEBUG: Calibration Peak: {max_test:.2f}")

        if max_test < 0.5:
            pass
        trx.sendMSG("Calibration Complete.")
        trx.sendMSG("Did the robot cross the black line? (If no, Reset)")
        calibration_successful = True
        time.sleep(1)
    # ==========================================================

    # --- DIAGNOSTIC: DID WE SEE BLACK? ---
    # We check the internal min/max values to see contrast
    # (Assuming ReflectiveArray has public min_vals/max_vals, if not we check a read)
    test_read = sensors.read_calibrated()
    max_test = max(test_read)
    trx.sendMSG(f"DEBUG: Peak Black Level Seen: {max_test:.2f}")
    if max_test < 0.5:
        trx.sendMSG("CRITICAL WARNING: Sensors barely see black! Check wiring/height.")
        trx.sendMSG("Check: 1. Sensor Height (too high?)")
        trx.sendMSG("       2. Connector loose?")
        trx.sendMSG("       3. Did robot spin over the black line?")
    else:
        trx.sendMSG("DEBUG: Sensors look healthy.")

    # --- WAIT FOR START ---
    if HAS_BUTTON:
        trx.sendMSG(">>> Press Button to Start... <<<")
        while start_button.value:
            time.sleep(0.1)
        while not start_button.value:
            time.sleep(0.1)
    else:
        trx.sendMSG("No button. Starting in 2s...")
        time.sleep(2)

    # --- INITIALIZE STATE ---
    pid = PID(kp=KP, ki=KI, kd=KD)
    track_index = 0
    bar_hits = 0

    current_track = TRACK_SEQUENCE[track_index]
    gaps_allowed = current_track["gaps_allowed"]

    last_time = time.monotonic()

    # TIMER FOR DEBUG PRINTS (Prevents freezing)
    last_debug_time = time.monotonic()

    trx.sendMSG(f"STARTING EVENT: {current_track['name']}")

    try:
        while True:
            # 1. Killswitch
            if HAS_BUTTON and not start_button.value:
                motors.stop()
                trx.sendMSG(">>> STOP BUTTON PRESSED")
                while not start_button.value:
                    time.sleep(0.1)
                break
            # 2. Timing
            now = time.monotonic()
            dt = now - last_time
            last_time = now
            if dt > MAX_DT:
                dt = MAX_DT

            # 3. Read Sensors
            vals = sensors.read_calibrated()

            # 4. Bar Logic (Leaky Bucket)
            if current_frame_is_bar(vals):
                bar_hits += 1
            else:
                bar_hits -= 1
                if bar_hits < 0:
                    bar_hits = 0
            # ==========================================================
            # SAFE DEBUGGING (Runs 2 times per second)
            # ==========================================================
            if now - last_debug_time > 0.5:
                # Calculate what the PID is trying to do
                debug_err = sensors.get_line_error()
                debug_max = max(vals)

                # Convert vals to 0-100 ints for easier reading
                # e.g., [5, 10, 99, 10, 5] means center sensor is on black
                # vals_int = [int(v * 100) for v in vals]

                msg = f"[{current_track['name'][:4]}] Hits:{bar_hits} | Max:{debug_max:.2f} | Err:{debug_err:.2f}"
                trx.sendMSG(msg)
                last_debug_time = now
            # ==========================================================

            # 5. Transition Logic
            if bar_hits >= 5:
                trx.sendMSG(f">>> TRANSITION: Leaving {current_track['name']}")

                # -------- CHECK ---------
                # Determine Blind Drive Time
                # If we are just starting, only do a tiny blip.
                if current_track["name"] == "START_LINE":
                    blind_time = START_CLEAR_TIME  # Uses the 0.1s variable
                else:
                    blind_time = BAR_CLEAR_TIME    # Uses the 0.6s variable
                # -----------------------------

                # Update Index
                track_index += 1
                if track_index >= len(TRACK_SEQUENCE):
                    trx.sendMSG(">>> SEQUENCE COMPLETE! Stopping.")
                    motors.stop()
                    break

                # Clear Bar
                trx.sendMSG(f"   -> Clearing Bar ({blind_time}s)...")
                motors.set_speeds(BASE_SPEED, BASE_SPEED)

                time.sleep(blind_time)

                motors.stop()
                # time.sleep(0.1)

                # Get New Track
                current_track = TRACK_SEQUENCE[track_index]
                gaps_allowed = current_track["gaps_allowed"]
                trx.sendMSG(f">>> ENTERING: {current_track['name']}")

                # Execute Action
                if current_track["action"] is not None:
                    trx.sendMSG(f"   -> Running Action: {current_track['name']}")
                    current_track["action"](motors, sensors)

                    # Reset PID after action
                    pid.reset()
                    bar_hits = 0
                    last_correction = 0.0
                    last_time = time.monotonic()
                    continue
                    
                # No Action (Reset and continue)
                pid.reset()
                bar_hits = 0
                last_correction = 0.0
                last_time = time.monotonic()
                continue
                
            # 6. Drive Logic
            if is_line_lost(vals):
                if gaps_allowed:
                    # HEADING LOCK WITH TRIM
                    l_speed = (BASE_SPEED - last_correction) * LEFT_TRIM_VAL
                    r_speed = (BASE_SPEED + last_correction) * RIGHT_TRIM_VAL
                    motors.set_speeds(l_speed, r_speed)
                time.sleep(0.001)
                continue
                
            # Normal PID
            err = sensors.get_line_error()
            correction = pid.update(0.0, err, dt)

            if correction > MAX_CORRECTION: correction = MAX_CORRECTION
            if correction < -MAX_CORRECTION: correction = -MAX_CORRECTION
            
            last_correction = correction

            l_speed = (BASE_SPEED - correction) * LEFT_TRIM_VAL
            r_speed = (BASE_SPEED + correction) * RIGHT_TRIM_VAL
            motors.set_speeds(l_speed, r_speed)
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        motors.stop()
        trx.sendMSG("Ctrl+C Stop")
    finally:
        motors.stop()


if __name__ == "__main__":
    run_robot()
