import time
import board
import digitalio
import random
from motor_2_channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID

# ================= SETUP =================
try:
    start_button = digitalio.DigitalInOut(board.D12)
    start_button.direction = digitalio.Direction.INPUT
    start_button.pull = digitalio.Pull.UP
    HAS_BUTTON = True
except:
    HAS_BUTTON = False

# ================= HELPER FUNCTIONS =================

def force_align_and_cross(motors, sensors, came_from):
    """
    Handles the arrival at a messy intersection based on the path taken.
    1. Forces a pivot based on 'came_from' to square up.
    2. Re-checks if line is present.
    3. Jolts forward until the line is CLEARED (sees white).
    """
    print(f">>> ARRIVAL LOGIC. Came from: {came_from}")
    
    # 1. Force Correction (Pivot to square up)
    # If we came from LEFT, we force LEFT to ensure we aren't angled into the mess.
    if came_from == 'LEFT':
        print("Forcing Left Alignment...")
        motors.set_speeds(-0.3, 0.3) # Spin Left
        time.sleep(0.25) # Short blind pivot
    elif came_from == 'RIGHT':
        print("Forcing Right Alignment...")
        motors.set_speeds(0.3, -0.3) # Spin Right
        time.sleep(0.25)
    
    motors.stop()
    time.sleep(0.1)

    # 2. Re-Check (Are we actually on the bar?)
    vals = sensors.read_calibrated()
    black_count = sum(1 for v in vals if v > 0.5)
    
    if black_count < 3:
        print("WARNING: Lost line after alignment pivot!")
        # Optional: wiggle back if needed, but for now we proceed or stop
    else:
        print("Alignment Confirmed. Line is solid.")

    # 3. Jolt Forward until WHITE (Clear the bar)
    print("Jolting forward to clear bar...")
    motors.set_speeds(0.35, 0.35)
    
    # Safety timeout in case we never see white
    jolt_start = time.monotonic()
    while time.monotonic() - jolt_start < 1.0:
        vals = sensors.read_calibrated()
        # Check if we see line (True if any sensor is black)
        see_line = any(v > 0.5 for v in vals)
        
        if not see_line:
            # We see all white! We have crossed the bar.
            print("Cleared bar (All White).")
            break
            
    motors.stop()
    time.sleep(0.2)

def standard_align(motors, sensors):
    """ Standard wiggle alignment for the START of a path (not arrival). """
    print(">>> STANDARD ALIGNMENT...")
    timeout = 2.0
    start_time = time.monotonic()
    
    while time.monotonic() - start_time < timeout:
        vals = sensors.read_calibrated()
        left_black = vals[0] > 0.5
        right_black = vals[7] > 0.5
        
        if left_black and right_black:
            motors.stop()
            return
        
        if left_black and not right_black:
            motors.set_speeds(-0.15, 0.15)
        elif right_black and not left_black:
            motors.set_speeds(0.15, -0.15)
        else:
            motors.set_speeds(0.15, 0.15)

    motors.stop()

def execute_random_fork(motors):
    choice = random.choice(['LEFT', 'CENTER', 'RIGHT'])
    print(f">>> DECISION: {choice}")
    
    TURN_SPEED = 0.4
    TURN_TIME = 0.4 
    FORWARD_TIME = 0.2
    
    if choice == 'LEFT':
        motors.set_speeds(0.0, TURN_SPEED)
        time.sleep(TURN_TIME)
    elif choice == 'RIGHT':
        motors.set_speeds(TURN_SPEED, 0.0)
        time.sleep(TURN_TIME)
    elif choice == 'CENTER':
        motors.set_speeds(TURN_SPEED, TURN_SPEED)
        time.sleep(FORWARD_TIME)
        
    return choice

def turn_180(motors, sensors):
    print(">>> EXECUTING 180 TURN")
    # Since we just "jolted" past the bar, the bar is BEHIND us.
    # We spin until we see it again.
    
    # 1. Blind Spin (get rotation started)
    motors.set_speeds(0.4, -0.4)
    time.sleep(0.4) 
    
    # 2. Spin until Center sees black
    timeout = 2.5
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        vals = sensors.read_calibrated()
        if vals[3] > 0.6 or vals[4] > 0.6:
            print("Line Acquired.")
            break
        motors.set_speeds(0.4, -0.4)
        
    motors.stop()
    time.sleep(0.2)

def run_fork_test(motors, sensors):
    KP = 0.40
    KI = 0.01
    KD = 0.055
    BASE_SPEED = 0.20
    
    pid = PID(kp=KP, ki=KI, kd=KD)
    
    MARKER_DEBOUNCE = 2.0
    
    # Track the last path taken so we know how to correct at the end
    last_choice = None 
    at_end_zone = False 

    try:
        while True:
            # --- SENSOR READ ---
            vals = sensors.read_calibrated()
            current_time = time.monotonic()
            
            # Intersection Detection
            black_count = sum(1 for v in vals if v > 0.6)
            outer_trigger = (vals[0] > 0.5 and vals[7] > 0.5)
            density_trigger = (black_count >= 6) # Slightly relaxed density
            
            is_intersection = (outer_trigger or density_trigger)

            # --- INTERSECTION LOGIC ---
            if is_intersection and (current_time - last_marker_time > MARKER_DEBOUNCE):
                motors.stop()
                print("Intersection Detected.")
                
                # CASE 1: ARRIVAL (We just finished a path)
                if last_choice is not None:
                    # Perform the specific "force and jolt" logic you requested
                    force_align_and_cross(motors, sensors, last_choice)
                    
                    # Now turn around
                    turn_180(motors, sensors)
                    
                    # We are now facing the fork again.
                    # We treat this as "Starting" a new path.
                    # We align normally to ensure we are straight before choosing.
                    standard_align(motors, sensors)
                    
                else:
                    # CASE 2: FIRST START (No path taken yet)
                    # Just push through slightly to get on the bar, then align
                    motors.set_speeds(BASE_SPEED, BASE_SPEED)
                    time.sleep(0.1)
                    standard_align(motors, sensors)

                # --- CHOOSE NEW PATH ---
                if not at_end_zone:
                    print("Status: Entering Fork")
                    last_choice = execute_random_fork(motors)
                    at_end_zone = True
                else:
                    print("Status: Leaving End Zone")
                    last_choice = execute_random_fork(motors)
                    at_end_zone = False 
                
                # Reset
                pid.reset()
                last_time = time.monotonic()
                last_marker_time = time.monotonic()
                continue

            # --- PID CONTROL ---
            dt = current_time - last_time
            last_time = current_time
            error = sensors.get_line_error()
            correction = pid.update(0.0, error, dt)
            motors.set_speeds(BASE_SPEED - correction, BASE_SPEED + correction)

    except KeyboardInterrupt:
        motors.stop()
