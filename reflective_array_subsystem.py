import board
import digitalio
import time

class ReflectiveArray:
    def __init__(self):
        # ----- IR emitter control -----
        # Initialize emitter pins (adjust pins as needed per your wiring)
        self.ctrl_odd = digitalio.DigitalInOut(board.D2)
        self.ctrl_odd.direction = digitalio.Direction.OUTPUT
        self.ctrl_odd.value = True 

        self.ctrl_even = digitalio.DigitalInOut(board.D11)
        self.ctrl_even.direction = digitalio.Direction.OUTPUT
        self.ctrl_even.value = True

        # ----- Sensor pins -----
        # Changes MADE HEREs
        # Assuming D10 is far left and D3 is far right:
        # Check documentation: 
        self.sensor_pins = [
            board.D10, board.D9, board.D8, board.D7, 
            board.D6, board.D5, board.D4, board.D3
        ]
        
        self.sensors = []
        for pin in self.sensor_pins:
            s = digitalio.DigitalInOut(pin)
            s.direction = digitalio.Direction.INPUT
            self.sensors.append(s)

        # Calibration data (min/max readings observed)
        self.min_vals = [200] * 8
        self.max_vals = [2000] * 8

    def read_raw(self, max_time_us=3000):
        """Reads raw decay times in microseconds."""
        # 1. Charge Capactitors
        for s in self.sensors:
            s.direction = digitalio.Direction.OUTPUT
            s.value = True
        
        time.sleep(0.00002) # 20us charge time

        # 2. Switch to Input and measure decay
        for s in self.sensors:
            s.direction = digitalio.Direction.INPUT
            # CircuitPython pull disabling is implicit on some boards or not required if external pullups absent

        start = time.monotonic_ns()
        max_time_ns = max_time_us * 1000
        times_ns = [None] * 8
        
        while True:
            now = time.monotonic_ns()
            elapsed = now - start
            all_done = True
            for i, s in enumerate(self.sensors):
                if times_ns[i] is None:
                    if not s.value: # pin went low
                        times_ns[i] = elapsed
                    else:
                        all_done = False
            
            if all_done or elapsed >= max_time_ns:
                break
        
        # Fill timeouts
        results = []
        for t in times_ns:
            if t is None:
                results.append(max_time_us)
            else:
                results.append(t // 1000)
        return results

    def read_calibrated(self):
        """Returns values normalized between 0.0 (white) and 1.0 (black)."""
        raw = self.read_raw()
        calibrated = []
        for i, val in enumerate(raw):
            # Update calibration dynamically (optional, but helps adaptation)
            self.min_vals[i] = min(self.min_vals[i], val)
            self.max_vals[i] = max(self.max_vals[i], val)
            
            denom = self.max_vals[i] - self.min_vals[i]
            if denom == 0: denom = 1
            
            # Normalize
            norm = (val - self.min_vals[i]) / denom
            calibrated.append(max(0.0, min(1.0, norm)))
            
        return calibrated

    def get_line_error(self):
        """
        Returns position error from -1.0 (left) to 1.0 (right).
        0.0 means centered.
        """
        vals = self.read_calibrated()
        
        # Weighted average method
        # Weights: -4, -3, -2, -1, +1, +2, +3, +4
        # We skip 0 to force a distinction between left and right
        weights = [-4, -3, -2, -1, 1, 2, 3, 4]
        
        numerator = 0.0
        denominator = 0.0
        
        for i in range(8):
            numerator += vals[i] * weights[i]
            denominator += vals[i]
            
        if denominator < 0.1: 
            return 0.0 # No line seen
            
        # Result is roughly -4 to 4, normalize to -1 to 1
        return (numerator / denominator) / 4.0