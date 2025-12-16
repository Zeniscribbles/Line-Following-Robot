import time
import board
import digitalio
from motor_2_channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID
from esp32_trx import TRX

# Import maneuvers (Only the complex ones)
import fork
import t_turn

# ---------------- WRAPPERS ----------------
def do_fork_action(motors, sensors, **kwargs):
    """Aligns, chooses a path, executes it, and saves the choice."""
    global LAST_FORK_CHOICE
    fork.standard_align(motors, sensors)
    LAST_FORK_CHOICE = fork.execute_random_fork(motors)

def fork_return_action(motors, sensors, **kwargs):
    """Uses the saved choice to align correctly and cross the bar."""
    global LAST_FORK_CHOICE
    if LAST_FORK_CHOICE is None:
        trx.sendMSG("WARNING: No choice saved. Defaulting to CENTER.")
        LAST_FORK_CHOICE = 'CENTER'
    
    fork.force_align_and_cross(motors, sensors, LAST_FORK_CHOICE)

# ---------------- CONFIGURATION ----------------
# --- UNIVERSAL TUNING (One reliable set) -------
KP = 0.40
KI = 0.01
KD = 0.055
BASE_SPEED = 0.25  # Safe speed. Increase if stable (e.g., 0.30)

# 1. TIMING & SENSITIVITY
MAX_DT = 0.05
MAX_CORRECTION = 0.45 
BAR_HITS_REQUIRED = 3     
BAR_CLEAR_TIME = 0.6      

# 2. SENSOR THRESHOLDS
BAR_THRESH = 0.80         # Black > 0.80
BAR_COUNT_THRESH = 6      # Bar = 6+ sensors black
GAP_THRESH = 0.10         # Line Lost = All sensors < 0.10 (White)

# 3. TRACK SEQUENCE
# "gaps_allowed": True = If line is lost, DRIVE STRAIGHT (Blind).
# "gaps_allowed": False = If line is lost, STOP (Safety).
TRACK_SEQUENCE = [
    {"name": "START_LINE",     "action": None,                "gaps_allowed": False},
    {"name": "SERPENTINE",     "action": None,                "gaps_allowed": False},
    {"name": "STRAIGHTAWAY",   "action": None,                "gaps_allowed": True},   # top-left bar
    {"name": "DO_TTURN",       "action": t_turn.run_t_turns,"gaps_allowed": False},
    {"name": "DO_FORK",        "action": do_fork_action,      "gaps_allowed": False},
    {"name": "FORK_RETURN",    "action": fork_return_action,  "gaps_allowed": False},
    
    {"name": "DO_TTURN",       "action": t_turn.run_t_turns,"gaps_allowed": False},
    {"name": "STRAIGHTAWAY",   "action": None,                "gaps_allowed": True},   # top-left bar
    {"name": "SERPENTINE",     "action": None,                "gaps_allowed": False},
    {"name": "END_SERP_RET",   "action": None,                "gaps_allowed": False},
    {"name": "FORK_RETURN",    "action": fork_return_action,  "gaps_allowed": False},

]


# -----------------------------------------------

trx = TRX(printDebug=False, blinkDebug=False)
trx.addPeer(bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]))

def current_frame_is_bar(vals):
    # Returns True if 6 or more sensors are seeing black (> 0.8)
    black_count = sum(1 for v in vals if v > BAR_THRESH)
    return black_count >= BAR_COUNT_THRESH

def is_line_lost(vals):
    """Returns True if sensors see all white (Gap)."""
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
        trx.sendMSG("Button initialized on D12")
    except Exception as e:
        trx.sendMSG(f"Button setup skipped: {e}")
        HAS_BUTTON = False

    # --- CALIBRATION ---
    trx.sendMSG("--- CALIBRATION ---")
    trx.sendMSG("Place robot ON THE TRACK (Spinning in 2s...)")
    time.sleep(2)
    motors.set_speeds(0.25, -0.25)
    start_cal = time.monotonic()
    while time.monotonic() - start_cal < 2.0:
        sensors.read_calibrated()
    motors.stop()
    trx.sendMSG("Calibration Done.")
    
    # REPORT SENSOR HEALTH
    # This tells you if the robot is actually seeing black!
    cal_read = sensors.read_calibrated()
    max_val = max(cal_read)
    trx.sendMSG(f"Calib Max: {max_val:.2f}")
    if max_val < 0.6:
        trx.sendMSG("WARNING: SENSORS NOT SEEING DARK BLACK!")
    
    # --- WAIT FOR START ---
    if HAS_BUTTON:
        trx.sendMSG(">>> Press Button to Start... <<<")
        while start_button.value: time.sleep(0.1) 
        while not start_button.value: time.sleep(0.1) 
    else:
        trx.sendMSG("No button. Starting in 2s...")
        time.sleep(2)
    
    # --- INITIALIZE STATE ---
    # Use Universal PID params
    pid = PID(kp=KP, ki=KI, kd=KD)
    
    track_index = 0
    bar_hits = 0

    current_track = TRACK_SEQUENCE[track_index]
    gaps_allowed = current_track["gaps_allowed"]

    last_time = time.monotonic()
    last_debug_time = time.monotonic() # Timer for printing

    trx.sendMSG(f"Ready. Next Event: {TRACK_SEQUENCE[0]['name']}")

    try:
        while True:
            # 1. Killswitch
            if HAS_BUTTON and not start_button.value:
                motors.stop()
                trx.sendMSG(">>> STOP BUTTON PRESSED")
                while not start_button.value: time.sleep(0.1)
                break

            # 2. Timing
            now = time.monotonic()
            dt = now - last_time
            last_time = now
            if dt > MAX_DT: dt = MAX_DT

            # 3. Read Sensors
            vals = sensors.read_calibrated()
            
            # ==========================================================
            # ROBUST BAR DETECTION (The Fix)
            # ==========================================================
            if current_frame_is_bar(vals):
                bar_hits += 1
            else:
                bar_hits -= 1
                if bar_hits < 0: bar_hits = 0
                
            # Debugging (ANTI-FREEZE: Only prints every 0.5s)
            if now - last_debug_time > 0.5:
                # format: [State] Hits: X | MaxSensor: 0.XX
                print(f"[{current_track['name']}] Hits:{bar_hits} | Max:{max(vals):.2f}")
                last_debug_time = now

            # TRIGGER TRANSITION
            if bar_hits >= 5:
                trx.sendMSG(f">>> BAR DETECTED! Leaving {current_track['name']}")
                
                # 1. Update State
                track_index += 1
                if track_index >= len(TRACK_SEQUENCE):
                    trx.sendMSG("Sequence Complete!")
                    motors.stop()
                    break

                # 2. CLEAR THE BAR (Crucial: Get off the line so we don't trigger again)
                #    We drive blind for a moment to get the sensors past the black line.
                trx.sendMSG("   -> Clearing Bar...")
                motors.set_speeds(BASE_SPEED, BASE_SPEED)
                time.sleep(BAR_CLEAR_TIME) 
                motors.stop()
                #time.sleep(0.2) # Adjust this time! (0.15 to 0.3 usually)
                
                # Optional: Ensure we are really off it (Safety check)
                # while is_black_bar(sensors.read_calibrated()):
                #     motors.set_speeds(BASE_SPEED, BASE_SPEED)
                
                motors.stop()
                time.sleep(0.1) # Stabilization pause

                # 3. GET NEW TRACK INFO
                current_track = TRACK_SEQUENCE[track_index]
                gaps_allowed = current_track["gaps_allowed"]

                trx.sendMSG(f">>> ENTERING: {current_track['name']}")

                
                # 4. EXECUTE SPECIAL ACTION (e.g., T-Turn or Fork)
                #    If the new state has a blocking action, run it NOW.
                if current_track["action"] is not None:
                    trx.sendMSG(f"   -> Executing Action for {current_track['name']}")
                    # Pass motors/sensors to the function (e.g., t_turn.run_t_turns)
                    current_track["action"](motors, sensors)
                    
                    # After action, reset variables for the NEXT segment
                    pid.reset()
                    bar_hits = 0
                    last_time = time.monotonic()
                    continue # Skip the rest of the loop, start fresh

                # 5. IF NO ACTION (Just a straightaway/serpentine):
                #    Reset variables and fall through to normal PID driving
                pid.reset()
                bar_hits = 0
                last_time = time.monotonic()
                continue 
            # ==========================================================

            # --- B. Check for Gaps (Line Lost) ---
            if is_line_lost(vals):
                if gaps_allowed:
                    # Drive Straight Blindly
                    # trx.sendMSG("In Gap - Driving Blind") 
                    motors.set_speeds(BASE_SPEED, BASE_SPEED)
                else:
                    # Safety Stop (We shouldn't lose the line here)
                    motors.set_speeds(0, 0)
                
                # Skip PID update this frame
                time.sleep(0.001)
                continue

            # --- C. Normal PID Line Following ---
            err = sensors.get_line_error()
            correction = pid.update(0.0, err, dt)
            
            if correction > MAX_CORRECTION: correction = MAX_CORRECTION
            if correction < -MAX_CORRECTION: correction = -MAX_CORRECTION
            
            motors.set_speeds(BASE_SPEED - correction, BASE_SPEED + correction)
            
            time.sleep(0.001)

    except KeyboardInterrupt:
        motors.stop()
        print("Ctrl+C Stop")
    finally:
        motors.stop()

if __name__ == "__main__":
    run_robot()