
"""
Author: Zach Nelson
"""
import time
import sys
import os

# Add library path
sys.path.append('/home/user/resources/TMotorCANControl/src')

from TMotorCANControl.mit_can import TMotorManager_mit_can
from SoftRealtimeLoop import SoftRealtimeLoop   # <-- Needed import

Type = 'AK10-9'
ID = 0


def ensure_can_up():
    os.system("sudo ip link set can0 up")


def speed_step(dev):
    dev.set_zero_position()
    time.sleep(1.5)  # Wait for the motor to zero

    dev.set_speed_gains(kd=3.0)
    print("Starting speed step demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)

    for t in loop:
        dev.update()

        if t < 1.0:
            dev.velocity = 0.0
        else:
            dev.velocity = 1.0
    
    del loop


if __name__ == '__main__':
    while True:
        try:
            print(f"Trying motor ID {ID}...")
            ensure_can_up()
            with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
                speed_step(dev)

        except Exception as e:
            print(f"ID {ID} didn't work: {e}")
            ID += 1

            if ID > 4:
                print("No valid motor IDs found. Exiting.")
                break

