import time
import board
import digitalio
import pwmio
from PID import PID  # <--- IMPORT YOUR PID CLASS

try:
    import countio
    _HAS_COUNTIO = True
except ImportError:
    _HAS_COUNTIO = False


# ======================= SIMPLE ENCODER CLASS ========================
class SimpleEncoder:
    def __init__(self, side, counts_per_rev=12, timeout=0.5):
        self.side = side
        self.cpr = counts_per_rev
        self.timeout = timeout

        # --- PINS DEFINED HERE ---
        if side == 'left':
            self._pin_id = board.D1
        elif side == 'right':
            self._pin_id = board.D0
        else:
            raise ValueError("Side must be 'left' or 'right'")

        self.last_time = time.monotonic()
        self.current_rpm = 0.0

        # Prefer countio if available
        self._counter = None
        self._last_count = 0

        if _HAS_COUNTIO:
            self._counter = countio.Counter(self._pin_id, edge=countio.Edge.RISE)
            self._last_count = self._counter.count
        else:
            self.pin = digitalio.DigitalInOut(self._pin_id)
            self.pin.direction = digitalio.Direction.INPUT
            self.last_state = self.pin.value
            self.count = 0

    def read_rpm(self):
        now = time.monotonic()
        dt = now - self.last_time

        if self._counter is not None:
            # countio path
            c = self._counter.count
            delta = c - self._last_count
            self._last_count = c

            if delta > 0:
                revs = delta / self.cpr
                self.current_rpm = (revs / dt) * 60.0
                self.last_time = now
            elif dt > self.timeout:
                self.current_rpm = 0.0

            return self.current_rpm

        # Polling fallback path
        val = self.pin.value
        if val and not self.last_state:
            self.count += 1
        self.last_state = val

        if self.count > 0:
            revs = self.count / self.cpr
            self.current_rpm = (revs / dt) * 60.0
            self.count = 0
            self.last_time = now
        elif dt > self.timeout:
            self.current_rpm = 0.0

        return self.current_rpm


# ========================== MOTOR DRIVER CLASS ==============================
class MotorDriver:
    def __init__(self):
        # --- MOTOR PINS ---
        # Left Motor
        self.dir_l = digitalio.DigitalInOut(board.A4)
        self.dir_l.direction = digitalio.Direction.OUTPUT
        self.pwm_l = pwmio.PWMOut(board.A6, frequency=20000, duty_cycle=0)

        # Right Motor
        self.dir_r = digitalio.DigitalInOut(board.A5)
        self.dir_r.direction = digitalio.Direction.OUTPUT
        self.pwm_r = pwmio.PWMOut(board.A7, frequency=20000, duty_cycle=0)

        # Sleep Pin
        self.slp = digitalio.DigitalInOut(board.A3)
        self.slp.direction = digitalio.Direction.OUTPUT
        self.slp.value = True # Enable driver

        # Create Encoders
        self.enc_l = SimpleEncoder(side='left')
        self.enc_r = SimpleEncoder(side='right')

        # --- RPM CONTROL CONFIGURATION ---
        # This is the RPM the robot travels at when speed is 1.0
        # If your robot is jerky, try lowering this. If it's too slow, raise it.
        self.MAX_RPM = 200.0 
        
        # PID Constants for RPM
        # These are different from Line Following!
        # kp=0.003 means for every 1 RPM error, we add 0.003 to the duty cycle.
        kp_rpm = 0.003 
        ki_rpm = 0.005
        kd_rpm = 0.0001
        
        self.pid_l = PID(kp_rpm, ki_rpm, kd_rpm, integral_limit=0.3)
        self.pid_r = PID(kp_rpm, ki_rpm, kd_rpm, integral_limit=0.3)
        
        self.last_loop_time = time.monotonic()

    def set_raw_duty(self, motor_char, duty):
        """
        Directly controls the hardware PWM. 
        Previously called 'set_speed', renamed to avoid confusion.
        """
        duty = max(-1.0, min(1.0, duty))
        speed_int = int(abs(duty) * 65535)
        is_forward = duty >= 0

        motor_char = motor_char.upper()
        if motor_char == 'L':
            self.dir_l.value = not is_forward
            self.pwm_l.duty_cycle = speed_int
        elif motor_char == 'R':
            self.dir_r.value = not is_forward
            self.pwm_r.duty_cycle = speed_int

    def set_speeds(self, left_input, right_input):
        """
        Uses PID to match the actual RPM to the requested input.
        left_input, right_input: -1.0 to 1.0 (fraction of MAX_RPM)
        """
        # 1. Measure Time Step (dt)
        now = time.monotonic()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        if dt <= 0: dt = 0.001

        # 2. Determine Target RPM
        target_l = left_input * self.MAX_RPM
        target_r = right_input * self.MAX_RPM

        # 3. Read Actual RPM
        # Encoders always return positive numbers, so we apply the sign 
        # based on what we are *trying* to do to keep math consistent.
        raw_rpm_l = self.enc_l.read_rpm()
        raw_rpm_r = self.enc_r.read_rpm()
        
        current_l = raw_rpm_l if left_input >= 0 else -raw_rpm_l
        current_r = raw_rpm_r if right_input >= 0 else -raw_rpm_r

        # 4. Calculate PID Correction
        # Feedforward: We guess the duty cycle (input) to start quickly
        # Output = Feedforward + PID_Correction
        pid_out_l = self.pid_l.update(target_l, current_l, dt)
        pid_out_r = self.pid_r.update(target_r, current_r, dt)

        final_duty_l = left_input + pid_out_l
        final_duty_r = right_input + pid_out_r

        # 5. Apply to Hardware
        self.set_raw_duty('L', final_duty_l)
        self.set_raw_duty('R', final_duty_r)

    def stop(self):
        self.set_raw_duty('L', 0)
        self.set_raw_duty('R', 0)
        # Reset PID internals so they don't "wind up" while stopped
        self.pid_l.reset()
        self.pid_r.reset()