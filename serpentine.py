# Serpentine: Medium Speed, High Derivative (snap to turns)
def get_params():
    return {
        "kp": 0.4,
        "ki": 0.01,
        "kd": 0.055,
        "speed": 0.45 # Slow down for the squiggles
    }