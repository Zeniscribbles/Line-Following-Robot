import time
import board
import digitalio
import pwmio

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

        # --- PINS DEFINED HERE INSTEAD OF GLOBALLY ---
        if side == 'left':
            self._pin_id = board.D1
        elif side == 'right':
            self._pin_id = board.D0
        else:
            raise ValueError("Side must be 'left' or 'right'")

        self.last_time = time.monotonic()
        self.current_rpm = 0.0

        # Prefer countio if available (won't miss pulses even if your loop is busy)
        self._counter = None
        self._last_count = 0

        if _HAS_COUNTIO:
            # Count rising edges
            self._counter = countio.Counter(self._pin_id, edge=countio.Edge.RISE)
            self._last_count = self._counter.count
        else:
            # Fallback: your original polling approach
            self.pin = digitalio.DigitalInOut(self._pin_id)
            self.pin.direction = digitalio.Direction.INPUT

            self.last_state = self.pin.value
            self.count = 0

    def read_rpm(self):
        now = time.monotonic()
        dt = now - self.last_time

        if dt <= 0:
            return self.current_rpm

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

        # Polling fallback path (original behavior)
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
        # --- MOTOR PINS DEFINED HERE ---
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

        # Create Encoders automatically
        self.enc_l = SimpleEncoder(side='left')
        self.enc_r = SimpleEncoder(side='right')

    def set_speed(self, motor_char, duty):
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
        else:
            raise ValueError("motor_char must be 'L' or 'R'")

    def set_speeds(self, left_duty, right_duty):
        self.set_speed('L', left_duty)
        self.set_speed('R', right_duty)
        
    def get_rpms(self):
        return (self.enc_l.read_rpm(), self.enc_r.read_rpm())

    def stop(self):
        self.set_speeds(0, 0)