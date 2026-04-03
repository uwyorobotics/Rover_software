'''Zach Nelson'''
#imports
import time
import sys
import os
#using the library: https://github.com/neurobionics/TMotorCANControl
#add to path# /home/user/resources/TMotorCANControl
sys.path.append('//home//user//resources//TMotorCANControl//src')
#/home/user/resources/TMotorCANControl/src/TMotorCANControl/__pycache__
from TMotorCANControl.mit_can import TMotorManager_mit_can

#instance the motors
#FrontLeftMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=1)
#FrontRightMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=1)
#BackLeftMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=2)
#BackRightMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=3)

#Node and totic definitions

#max speed in radians per second
#speed = 10000

'''
def spinMotor(point):
    radpersec = point
    FrontLeftMotor.set_motor_velocity_radians_per_second(radpersec)
    #FrontRightMotor.set_motor_velocity_radians_per_second(radpersec)
    #BackLeftMotor.set_motor_velocity_radians_per_second(radpersec)
    #BackRightMotor.set_motor_velocity_radians_per_second(radpersec)


#run the code in an infinite loop
spinMotor(1)
wait(10)
spinMotor(0)
'''


Type = 'AK10-9'
ID = 0

def speed_step(dev):
    dev.set_zero_position()
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    dev.set_speed_gains(kd=3.0)

    print("Starting speed step demo. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
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
            with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
                 speed_step(dev)
        except:
            print(f'{ID} didnt work, trying {ID+1}')
            ID = ID+1
            if ID > 4: 
                break



