import time
import board
import digitalio
import random

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
        motors.set_speeds(0.0, 0.40)  # Spin Left
        time.sleep(0.15)  # Short blind pivot
    elif came_from == 'RIGHT':
        print("Forcing Right Alignment...")
        motors.set_speeds(0.40, 0.0)  # Spin Right
        time.sleep(0.15)
    
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
    motors.set_speeds(0.25, 0.25)
    
    # Safety timeout in case we never see white
    jolt_start = time.monotonic()
    while time.monotonic() - jolt_start < 0.5:
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
    # choice = random.choice(['LEFT', 'CENTER', 'RIGHT'])
    choice = 'CENTER'
    print(f">>> DECISION: {choice}")
    
    TURN_SPEED = 0.4
    TURN_TIME = 0.2 
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
    motors.set_speeds(0.3, -0.3)
    time.sleep(0.5) 

    # 2. Spin until we see WHITE (Clear the messy intersection/bar)
    # This prevents the robot from stopping instantly because it saw the bar it just crossed.
    print("   -> Waiting for clear (White)...")
    timeout = 1.0
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        vals = sensors.read_calibrated()
        # If middle sensors are basically white (< 0.5), we are clear
        if vals[3] < 0.5 and vals[4] < 0.5:
            break
        # Keep spinning
        motors.set_speeds(0.25, -0.25)
    
    # 3. Spin until Center sees BLACK (Acquire new line)
    print("   -> Searching for Line (Black)...")
    timeout = 0.5
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        vals = sensors.read_calibrated()
        if vals[0] > 0.6 and vals[7] > 0.6:
            print("Line Acquired.")
            break
        motors.set_speeds(0.25, -0.25)
        
    motors.stop()
    time.sleep(0.2)

    # ====================================================
    # 4. NEW: CREEP RECOVERY (If we missed the line)
    # ====================================================
    vals = sensors.read_calibrated()
    # If max sensor value is low (< 0.5), we are on white (LOST)
    if max(vals) < 0.5:
        print(">>> RECOVERY: Line missed. Creeping...")
        
        # Try 4 times: Scan -> Creep -> Check
        for i in range(4):
            # A. Small Wiggle (Spin) to check side-to-side
            print(f"   -> Scan {i+1}...")
            
            # Scan Left
            motors.set_speeds(-0.15, 0.15)
            time.sleep(0.15)
            if max(sensors.read_calibrated()) > 0.5:
                break
            
            # Scan Right (past center)
            motors.set_speeds(0.15, -0.15)
            time.sleep(0.3)
            if max(sensors.read_calibrated()) > 0.5:
                break
            
            # Re-center
            motors.set_speeds(-0.15, 0.15)
            time.sleep(0.15)
            
            # B. Creep Forward
            print(f"   -> Creep {i+1}...")
            motors.set_speeds(0.10, 0.10)  # Slow Creep
            time.sleep(0.25)
            motors.stop()
            
            # Check again
            if max(sensors.read_calibrated()) > 0.5: 
                print("   -> Line Found!")
                break

