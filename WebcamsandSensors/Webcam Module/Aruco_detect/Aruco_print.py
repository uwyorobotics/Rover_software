#1/usr/bin/env python3
#tell ross that this is a python source file
#########################################
#Write the detected aruco tags on a new terminal
#listen to the Detect_topic topic and log it 
import rospy
from std_msgs.msg import String
detect_topic = 'detect'
publisherNodename = 'aruco_out'
detect_topic = 'detect'

def callbackFunction(message):
	rospy.loginfo("Aruco_Detect detected a tag")
	rospy.loginfo(str(message))
	
#init subscriber node
rospy.init_node(publisherNodename, anonymous = True)
#create the subscriber object
rospy.Subscriber(detect_topic, String, callbackFunction) #subscribe 
rospy.spin()#runs code in inf loop
	
	
