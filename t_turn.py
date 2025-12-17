import time
import board
import digitalio

from motor_2_channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID

# ================= CONFIGURATION =================
TURN_SPEED          = 0.4
BASE_SPEED          = 0.15
CORNER_SENSITIVITY  = 0.8
ALL_WHITE_THRESHOLD = 0.2
MEMORY_THRESHOLD    = 0.5
HARD_TURN_DURATION  = 0.3

# --- EXIT CONDITIONS ---
BAR_THRESH          = 0.60     # Matched to robot_main
BAR_COUNT_THRESH    = 4        # Matched to robot_main
BAR_HITS_REQUIRED   = 3        # How many times we confirm the bar
DEBUG_INTERVAL      = 0.2
# =================================================


# --------------- SENSOR STATE --------------------
def read_line_state(sensors, last_valid_error):
    vals = sensors.read_calibrated()
    max_reflection = max(vals)
    is_lost = (max_reflection < ALL_WHITE_THRESHOLD)

    if max_reflection > MEMORY_THRESHOLD:
        last_valid_error = sensors.get_line_error()

    return vals, max_reflection, is_lost, last_valid_error

def current_frame_is_bar(vals):
    """Checks if the CURRENT frame looks like a bar."""
    count = sum(1 for v in vals if v >= BAR_THRESH)
    return count >= BAR_COUNT_THRESH

# -------------- CONTROL LOGIC --------------------
def apply_turn_lock(motors, now, turn_lock_until, turn_lock_direction):
    if now < turn_lock_until:
        if turn_lock_direction == -1:
            motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            motors.set_speeds(TURN_SPEED, -TURN_SPEED)
        return True
    return False

def trigger_hard_turn_if_needed(motors, vals, now):
    # =========================================================
    # CRITICAL FIX: THE BAR GUARD
    # =========================================================
    # If BOTH the far-left and far-right sensors are black, 
    # we are hitting the EXIT BAR. Do NOT turn.
    if vals[0] > CORNER_SENSITIVITY and vals[7] > CORNER_SENSITIVITY:
        return False, 0.0, 0
    # =========================================================

    if vals[0] > CORNER_SENSITIVITY:
        print(f"   >>> [T-TURN] LEFT INTERSECTION (Sens0: {vals[0]:.2f})")
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
        return True, now + HARD_TURN_DURATION, -1

    if vals[7] > CORNER_SENSITIVITY:
        print(f"   >>> [T-TURN] RIGHT INTERSECTION (Sens7: {vals[7]:.2f})")
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)
        return True, now + HARD_TURN_DURATION, 1

    return False, 0.0, 0

def handle_lost_line(motors, last_valid_error):
    if last_valid_error < 0:
        motors.set_speeds(-TURN_SPEED, TURN_SPEED)
    else:
        motors.set_speeds(TURN_SPEED, -TURN_SPEED)

def pid_drive(motors, sensors, pid, dt):
    err = sensors.get_line_error()
    correction = pid.update(0.0, err, dt)
    motors.set_speeds(BASE_SPEED - correction, BASE_SPEED + correction)


# ------------------ MAIN LOOP --------------------
def run_line_follower(motors, sensors, pid):
    last_valid_error = 0.0
    last_time = time.monotonic()
    
    # FIX 1: Initialize this variable so it doesn't crash!
    last_debug_time = time.monotonic() 

    turn_lock_until = 0.0
    turn_lock_direction = 0  
    
    # FIX 2: Re-enable the counter for robustness
    bar_hits = 0 

    print(">>> T-TURN STARTED: Entering Loop...")
    
    bar_hits = 0

    while True:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            # 1. Read Sensors
            vals, max_reflection, is_lost, last_valid_error = read_line_state(sensors, last_valid_error)
            
            # --- DEBUG PRINTS ---
            if now - last_debug_time > DEBUG_INTERVAL:
                sens_str = "".join(["#" if v > 0.5 else "_" for v in vals])
                print(f"[T-TURN] Hits:{bar_hits} | Max:{max_reflection:.2f} | {sens_str}")
                last_debug_time = now
            # --------------------

            # 2. EXIT CONDITION: This returns control to robot_main
            # FIX 3: Use the "Leaky Bucket" logic again
            if current_frame_is_bar(vals):
                bar_hits += 1
            else:
                bar_hits -= 1
                if bar_hits < 0: bar_hits = 0
            
            if bar_hits >= BAR_HITS_REQUIRED:
                print(f">>> T-TURN COMPLETE: Bar detected ({bar_hits} hits).")
                motors.stop()
                return # SUCCESS: We go back to the Main Menu

            # 3. Turn Lock
            if apply_turn_lock(motors, now, turn_lock_until, turn_lock_direction):
                continue

            # 4. Check for Intersections (Now includes Bar Guard)
            triggered, new_until, new_dir = trigger_hard_turn_if_needed(motors, vals, now)
            if triggered:
                turn_lock_until = new_until
                turn_lock_direction = new_dir
                continue

            # 5. Lost Line Logic
            if is_lost:
                handle_lost_line(motors, last_valid_error)
                continue

            # 6. Standard PID
            pid_drive(motors, sensors, pid, dt)

# ================== PUBLIC ENTRYPOINT ==================
def run_t_turns(motors, sensors, **kwargs):
    kp = kwargs.get('kp', 0.40)
    ki = kwargs.get('ki', 0.01)
    kd = kwargs.get('kd', 0.055)

    pid = PID(kp=kp, ki=ki, kd=kd)

    try:
        run_line_follower(motors, sensors, pid)
    except KeyboardInterrupt:
        motors.stop()
        raise