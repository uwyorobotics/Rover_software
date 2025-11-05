#1/usr/bin/env python3
#tell ross that this is a python source file

import rospy
from sensor_msgs.msg import Image
#import the bridge
from cv_bridge import CvBridge

import cv2

subscriberNodename = 'cam_sub'

topicName = 'videoTopic'

def callbackFunction(message):
	bridgeObject = CvBridge()
	
	rospy.loginfo("received a video message/frame")
	
	convertedframeBackToCV = bridgeObject.imgmsg_to_cv2(message)
	
	cv2.imshow("camera", convertedframeBackToCV)
	
	cv2.waitKey(1)

#init subscriber node
rospy.init_node(subscriberNodename, anonymous = True)

rospy.Subscriber(topicName, Image, callbackFunction)

rospy.spin()#runs code in inf loop

cv2.destroyAllWindows()



