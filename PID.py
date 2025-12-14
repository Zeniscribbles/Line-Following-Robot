class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement

        # integral
        self._integral += error * dt
        if self.integral_limit is not None:
            if self._integral > self.integral_limit:
                self._integral = self.integral_limit
            elif self._integral < -self.integral_limit:
                self._integral = -self.integral_limit

        # derivative
        if self._first or dt <= 0:
            derivative = 0.0
            self._first = False
        else:
            derivative = (error - self._prev_error) / dt

        self._prev_error = error

        return (
            self.kp * error +
            self.ki * self._integral +
            self.kd * derivative
        )