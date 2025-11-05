'''Zach Nelson
4/21/2025
Keystroke listener for TeleOP for rover
Outline: 
    instance ROS Node
        Loop{
        Listen for key strokes
        send keystroke to TOPIC
        }           '''

'''
wasd move forward, left, back, right
m - increase speed
n - decrease speed'''



#Use Rospy
import rospy
#use <> as a vector mesage
from geometry_msgs.msg import Point
#use pynput to listen to the keyboard
from pynput import keyboard

#global Variable speed
speed = 0

#whenever a key is pressed, make sure the key is valid, then translate it to a vector
#the TranslateToVector also sends the vector to the ROS Topic
def on_press(key):
    try:
        print('alphanumeric key {} pressed'.format(key.char))
        TranslateToVector(key)
    except AttributeError:
        print('special key {} pressed'.format(key))
    
#only allow the rover to drive when a key is actively being pressed
#this disallows the rover to run away in case something goes wrong
#the TranslateToVector also sends the vector to the ROS Topic
def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        listener.stop()
        return False
    key = "Stop"
    TranslateToVector(key)

def TranslateToVector(thisKey):
    match thisKey:
            case "Stop": 
                speed = 0
                Vector = Point(0,0,0)
            case 'w': #move forward
                speed = .5
                Vector = Point(speed,0,0)
            case 's': #move backwards
                speed = -.5
                Vector = Point(speed,0,0)
            case 'a':
                Vector = Point(0,.5,0)
            case 'd':
                Vector = Point(0,-.5,0)
            case 'm':
                speed = speed + .1
            case 'n':
                speed = speed - .1
    #publish to the topic
    publisher.publish(Vector)
    #wait until frequency 
    rate.sleep()

#Node and topic definitions
publisherNodeName = 'KeyPublisher'
topicName = 'motorTopic'

#initialize the publisher node
rospy.init_node(publisherNodeName, anonymous = True)

#create the publisher object
publisher = rospy.Publisher(topicName, Point ,queue_size = 60)

#define the rate
rate = rospy.Rate(30)


while not rospy.is_shutdown():
        #listen for a keystroke
        with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
            listener.start()
            #this should run as an infinite loop, always listening to the keyboard
            #when a key is pressed, the callback function sends the key to the TranslateToVector Function
            #which sends it to the ros topic after translating it to the propper format
