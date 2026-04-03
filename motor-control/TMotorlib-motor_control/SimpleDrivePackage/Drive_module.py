'''Zach Nelson
4/21/2025
Updating Simple speed control scipt 
Outline: 
    instance ROS Node
    Instance Motors on CAN
    
        Loop{
        Listen to a ROS topic
        update motor speed
        } '''

#imports
#using ROS
import rospy 
#position (drive commands)are encoded as a Point
from geometry_msgs.msg import Point
import time
#using the library: https://github.com/neurobionics/TMotorCANControl
from TMotorCANControl.TMotorManager_servo_can import TMotorManager_servo_can

#instance the motors
FrontLeftMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=0)
FrontRightMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=1)
BackLeftMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=2)
BackRightMotor = TMotorManager_servo_can(motor_type='AK10-9', motor_ID=3)

#Node and totic definitions
subscriberNodename = 'Motor_Driver'
topicName = 'motorTopic'

#max speed in radians per second
speed = 10000


def callbackFunction(point):
    rospy.loginfo("received a video message/frame")
    X = point.X
    Y = point.Y
    if X != 0:
        radpersec = speed*X
    elif Y != 0:
        radpersec = speed*Y*.25
    else:
        radpersec = 0
    FrontLeftMotor.set_motor_velocity_radians_per_second(radpersec)
    FrontRightMotor.set_motor_velocity_radians_per_second(radpersec)
    BackLeftMotor.set_motor_velocity_radians_per_second(radpersec)
    BackRightMotor.set_motor_velocity_radians_per_second(radpersec)

#Instance the ROS Node
rospy.init_node(subscriberNodename, anonymous = True)

#Listen to the Motot Topic
rospy.Subscriber(topicName, Point, callbackFunction)

#run the code in an infinite loop
rospy.spin()