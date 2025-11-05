'''download libraries!
pip install :pip install TMotorCANControl'''
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE! we are using AK10-9
Type = 'AK10-9'
ID = 1  ## will be 1-4

def speed_step(dev):
    dev.set_zero_position()
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    dev.set_speed_gains(kd=3.0)
    print("Starting speed step demo. Press ctrl+C to quit.")
    seconds = range(0,10)
    for t in seconds:
        dev.update()
        if t < 5.0:
            dev.velocity = 0.0
        else:
            dev.velocity = 1.0
        time.sleep(1.0)
    #tell the motor to stop, shut off, and end the program

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        speed_step(dev)