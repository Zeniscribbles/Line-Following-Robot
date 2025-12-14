# Serpentine: Medium Speed, High Derivative (snap to turns)
def get_params():
    return {
        "kp": 1.5,
        "ki": 0.0,
        "kd": 0.8,
        "speed": 0.45 # Slow down for the squiggles
    }