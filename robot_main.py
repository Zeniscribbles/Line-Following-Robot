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

# ---------------- CONFIGURATION ----------------
# --- UNIVERSAL TUNING (One reliable set) ---
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
    # Event 0: START LINE (Start on bar -> Go immediately)
    # Enters Serpentine -> NO GAPS ALLOWED in squiggles.
    {"name": "START_LINE",       "action": None, "gaps_allowed": False},
    
    # Event 1: End of Serpentine (Entering Straightaway)
    # Enters Straight -> GAPS ALLOWED (Blind Drive OK).
    {"name": "END_SERPENTINE",   "action": None, "gaps_allowed": True},
    
    # Event 2: T-Turn Left (Entering Straight)
    {"name": "DO_TTURN_LEFT",    "action": t_turn.execute_t_turn, "args": {"turn_left": True}, "gaps_allowed": True},
    
    # Event 3: Fork (Entering Serpentine/Complex)
    {"name": "DO_FORK",          "action": fork.handle_fork, "args": {"direction": "random"}, "gaps_allowed": False},
    
    # Event 4: U-Turn (Returning)
    {"name": "DO_UTURN",         "action": "UTURN_FUNC", "gaps_allowed": False}, 

    # --- RETURN PASS ---
    # Event 5: Fork Return (Entering Straight)
    {"name": "FORK_RETURN",      "action": None, "gaps_allowed": True},
    
    # Event 6: T-Turn Right (Entering Straight)
    {"name": "DO_TTURN_RIGHT",   "action": t_turn.execute_t_turn, "args": {"turn_left": False}, "gaps_allowed": True},
    
    # Event 7: Start Serpentine Return (Entering Serpentine)
    {"name": "START_SERP_RET",   "action": None, "gaps_allowed": False},
    
    # Event 8: End Serpentine Return (Entering Straight Dash)
    {"name": "END_SERP_RET",     "action": None, "gaps_allowed": True},
    
    # Event 9: Finish Line
    {"name": "FINISH",           "action": "STOP", "gaps_allowed": False}
]
# -----------------------------------------------

trx = TRX(printDebug=False, blinkDebug=False)
trx.addPeer(bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]))

def is_black_bar(vals):
    """Returns True if it sees a thick black bar."""
    count = sum(1 for v in vals if v >= BAR_THRESH)
    return count >= BAR_COUNT_THRESH

def is_line_lost(vals):
    """Returns True if sensors see all white (Gap)."""
    return max(vals) < GAP_THRESH

def execute_u_turn(motors, sensors):
    """Executes ~180-degree turn."""
    trx.sendMSG("Exec U-Turn")
    print("Exec U-Turn")

    TURN_SPEED = 0.35
    MIN_SPIN_TIME = 0.6
    MAX_SPIN_TIME = 4.0
    WHITE_SUM_THRESH = 1.0 
    ACQUIRE_SAMPLES = 6

    motors.stop()
    time.sleep(0.1)

    # Spin Left
    motors.set_speeds(-TURN_SPEED, TURN_SPEED)

    spin_start = time.monotonic()
    saw_white = False
    stable_count = 0

    while True:
        elapsed = time.monotonic() - spin_start
        if elapsed > MAX_SPIN_TIME:
            print("U-Turn Timeout")
            break

        vals = sensors.read_calibrated()
        
        # 1. Wait until we leave the current black line
        if sum(vals) <= WHITE_SUM_THRESH:
            saw_white = True

        # 2. Gate re-acquisition
        if elapsed < MIN_SPIN_TIME or not saw_white:
            continue

        # 3. Check for new line centered
        center_hit = (vals[3] > 0.5 or vals[4] > 0.5)
        line_mass = sum(1 for v in vals if v > 0.5)
        
        if center_hit and (1 <= line_mass <= 4):
            stable_count += 1
            if stable_count >= ACQUIRE_SAMPLES:
                break 
        else:
            stable_count = 0

    motors.stop()
    time.sleep(0.1)
    print("U-Turn Complete")

def run_robot():
    # --- HARDWARE SETUP ---
    motors = MotorDriver()
    sensors = ReflectiveArray()
    
    try:
        start_button = digitalio.DigitalInOut(board.D12)
        start_button.direction = digitalio.Direction.INPUT
        start_button.pull = digitalio.Pull.UP
        HAS_BUTTON = True
        print("Button initialized on D12")
    except Exception as e:
        print(f"Button setup skipped: {e}")
        HAS_BUTTON = False

    # --- CALIBRATION ---
    print("--- CALIBRATION ---")
    print("Place robot ON THE TRACK (Spinning in 2s...)")
    time.sleep(2)
    motors.set_speeds(0.25, -0.25)
    start_cal = time.monotonic()
    while time.monotonic() - start_cal < 2.0:
        sensors.read_calibrated()
    motors.stop()
    print("Calibration Done.")
    
    # --- WAIT FOR START ---
    if HAS_BUTTON:
        print(">>> Press Button to Start... <<<")
        while start_button.value: time.sleep(0.1) 
        while not start_button.value: time.sleep(0.1) 
    else:
        print("No button. Starting in 2s...")
        time.sleep(2)
    
    # --- INITIALIZE STATE ---
    # Use Universal PID params
    pid = PID(kp=KP, ki=KI, kd=KD)
    
    track_index = 0
    bar_hits = 0
    # Initially False because we start on a bar/Serpentine
    gaps_allowed = False 
    last_time = time.monotonic()

    print(f"Ready. Next Event: {TRACK_SEQUENCE[0]['name']}")

    try:
        while True:
            # 1. Killswitch
            if HAS_BUTTON and not start_button.value:
                motors.stop()
                print(">>> STOP BUTTON PRESSED")
                while not start_button.value: time.sleep(0.1)
                break

            # 2. Timing
            now = time.monotonic()
            dt = now - last_time
            last_time = now
            if dt > MAX_DT: dt = MAX_DT

            # 3. Read Sensors
            vals = sensors.read_calibrated()

            # --- A. Check for Black Bar (Sequence Trigger) ---
            if is_black_bar(vals):
                bar_hits += 1
            else:
                bar_hits = 0

            if bar_hits >= BAR_HITS_REQUIRED:
                print(f">>> BAR DETECTED: Triggering {TRACK_SEQUENCE[track_index]['name']}")
                motors.stop()
                time.sleep(0.05)
                
                current_event = TRACK_SEQUENCE[track_index]
                action = current_event["action"]
                gaps_allowed = current_event["gaps_allowed"] # Update Gap Logic
                args = current_event.get("args", {})

                if action == "STOP":
                    print("FINISH LINE")
                    break
                elif action == "UTURN_FUNC":
                    execute_u_turn(motors, sensors)
                elif callable(action):
                    action(motors, sensors, **args)
                
                # --- CLEAR BAR (Drive Blindly Past It) ---
                if action is None:
                    print("Blind Drive (Clearing Bar)...")
                    motors.set_speeds(BASE_SPEED, BASE_SPEED)
                    time.sleep(BAR_CLEAR_TIME) 
                    # Ensure sensors see white/line before re-engaging logic
                    while is_black_bar(sensors.read_calibrated()):
                        motors.set_speeds(BASE_SPEED, BASE_SPEED)
                        time.sleep(0.05)
                
                track_index += 1
                if track_index >= len(TRACK_SEQUENCE):
                    print("Sequence Complete!")
                    break

                pid.reset()
                bar_hits = 0
                last_time = time.monotonic()
                continue 

            # --- B. Check for Gaps (Line Lost) ---
            if is_line_lost(vals):
                if gaps_allowed:
                    # Drive Straight Blindly
                    # print("In Gap - Driving Blind") 
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