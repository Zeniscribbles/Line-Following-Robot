import board
import digitalio

class ReflectiveArray:
    def __init__(self, line_high=True, pull=None, ema_alpha=None):
        """
        line_high:
            True  -> sensor.value == True means "black"
            False -> sensor.value == False means "black" (common on many modules)

        pull:
            None, digitalio.Pull.UP, or digitalio.Pull.DOWN (only if your sensor output floats)

        ema_alpha:
            None (no filtering) or float in (0,1]. Example: 0.35
            Higher = more responsive, lower = smoother.
        """
        self.sensor_pins = [
            board.D10, board.D9, board.D8, board.D7,
            board.D6, board.D5, board.D4, board.D3
        ]

        self.sensors = []
        for pin in self.sensor_pins:
            s = digitalio.DigitalInOut(pin)
            s.direction = digitalio.Direction.INPUT
            if pull is not None:
                s.pull = pull
            self.sensors.append(s)

        # "Calibration" tracking (with digital sensors, this will just become 0/1)
        self.min_vals = [1] * 8
        self.max_vals = [0] * 8

        # Emitter control pins
        self.ctrl_odd = digitalio.DigitalInOut(board.D2)
        self.ctrl_odd.direction = digitalio.Direction.OUTPUT
        self.ctrl_odd.value = True

        self.ctrl_even = digitalio.DigitalInOut(board.D11)
        self.ctrl_even.direction = digitalio.Direction.OUTPUT
        self.ctrl_even.value = True

        self.line_high = line_high

        # Lost-line handling
        self._last_error = 0.0

        # Optional smoothing for jittery digital readings
        self.ema_alpha = ema_alpha
        self._filtered_error = 0.0

        # Optional black-bar debounce state
        self._bar_hits = 0

    def read_raw(self):
        """Raw boolean reads from the digital pins."""
        return [s.value for s in self.sensors]

    def read_calibrated(self):
        """
        Returns a list of ints (0/1) where 1 means "black".
        Also updates min/max for your main's calibration printout.
        """
        raw = self.read_raw()

        if self.line_high:
            vals = [1 if v else 0 for v in raw]
        else:
            vals = [0 if v else 1 for v in raw]

        # Update min/max (digital-only will converge to 0/1, but at least it's consistent)
        for i, v in enumerate(vals):
            if v < self.min_vals[i]:
                self.min_vals[i] = v
            if v > self.max_vals[i]:
                self.max_vals[i] = v

        return vals

    def get_line_error(self):
        """
        Returns -1.0 (Left) to 1.0 (Right). 0.0 is center.
        If line is lost (no black detected), returns the last known error.
        """
        vals = self.read_calibrated()

        # Weights: left negative, right positive
        weights = [-4, -3, -2, -1, 1, 2, 3, 4]

        num = 0
        den = 0
        for i, v in enumerate(vals):
            if v:
                num += weights[i]
                den += 1

        if den == 0:
            # Lost line: keep turning the same way as last seen
            err = self._last_error
        else:
            err = (num / den) / 4.0
            self._last_error = err

        # Optional smoothing to reduce twitch with digital sensors
        if self.ema_alpha is not None:
            a = self.ema_alpha
            self._filtered_error = a * err + (1.0 - a) * self._filtered_error
            return self._filtered_error

        return err

    def black_count(self):
        """How many sensors currently see black (1)."""
        return sum(self.read_calibrated())

    def is_black_bar(self, threshold=6):
        """
        Instantaneous check: True if >= threshold sensors see black.
        (Your updated main already does consecutive-hit confirmation.)
        """
        return self.black_count() >= threshold

    def reset_bar_detector(self):
        self._bar_hits = 0

    def is_black_bar_confirmed(self, threshold=6, required_hits=3):
        """
        Debounced black bar detector (optional).
        Use this if you want the debounce inside the sensor class instead of main.
        """
        if self.is_black_bar(threshold=threshold):
            self._bar_hits += 1
        else:
            self._bar_hits = 0
        return self._bar_hits >= required_hits
