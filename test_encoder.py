import time
import board
from motor_2_channel import MotorDriver

# Initialize the driver (which initializes the encoders)
driver = MotorDriver()

print("------------------------------------------------")
print("ENCODER TEST - Spin wheels by hand!")
print("If you see 0.0, your encoder pins are wrong.")
print("------------------------------------------------")

while True:
    # Read RPMs directly
    rpm_l, rpm_r = driver.get_rpms()
    
    # We also print the raw internal count if possible, to debug connections
    # (Accessing internal hidden variables just for this test)
    if driver.enc_l._counter:
        raw_l = driver.enc_l._counter.count
    else:
        raw_l = driver.enc_l.count
        
    if driver.enc_r._counter:
        raw_r = driver.enc_r._counter.count
    else:
        raw_r = driver.enc_r.count

    print(f"L_RPM: {rpm_l:.1f} (Ticks: {raw_l})  |  R_RPM: {rpm_r:.1f} (Ticks: {raw_r})")
    time.sleep(0.2)