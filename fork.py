import time
import board
import digitalio
from motor_driver_subsystem_2channel import MotorDriver
from reflective_array_subsystem import ReflectiveArray
from PID import PID
from esp32_trx import TRX

# ================= CONFIGURATION =================
TURN_SPEED          = 0.4   # Speed for hard 90-degree turns
BASE_SPEED          = 0.35  # Normal driving speed
CORNER_SENSITIVITY  = 0.8   # 0.0 to 1.0 (How dark edge must be to trigger hard turn)
ALL_WHITE_THRESHOLD = 0.2   # If all sensors are below this, we are lost
MEMORY_THRESHOLD    = 0.5   # Only update memory if signal is stronger than this
HARD_TURN_DURATION  = 0.3   # Seconds to lock into a hard turn (blind)

trx = TRX(printDebug = False, blinkDebug = False)
trx.addPeer(bytes([0xec, 0xda, 0x3b, 0x61, 0x58, 0x58]))

SEND_TRX = True
# =================================================

# Button Setup
try:
    start_button = digitalio.DigitalInOut(board.D12)
    start_button.direction = digitalio.Direction.INPUT
    start_button.pull = digitalio.Pull.UP
    HAS_BUTTON = True
except:
    HAS_BUTTON = False
