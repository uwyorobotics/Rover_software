#1/usr/bin/env python3
#tell ross that this is a python source file

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
#import the bridge
from cv_bridge import CvBridge
import cv2

# subscriber and publisher definitions
subscriberNodename = 'cam_sub'
topicName = 'videoTopic'
detect_topic = 'detect'
#publisher stuff
publisher = rospy.Publisher(detect_topic,String,queue_size = 60)
#aruco Junk
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


def callbackFunction(message):
	bridgeObject = CvBridge()
	rospy.loginfo("received a video message/frame")
	convertedframeBackToCV = bridgeObject.imgmsg_to_cv2(message)
	
	#do the detect
	#Convert the image to grayscale
	gray = cv2.cvtColor(convertedframeBackToCV, cv2.COLOR_BGR2GRAY)
	corners, ids, rejected = detector.detectMarkers(gray)

	if ids is not None:
		message = "Detected markers: " +  str(ids)
		rospy.loginfo(message)
		message = str(ids)
		publisher.publish(message)
		convertedframeBackToCV = np.array(convertedframeBackToCV)
		cv2.aruco.drawDetectedMarkers(convertedframeBackToCV, corners, ids)
	cv2.imshow('detected Markers', convertedframeBackToCV)
	
	cv2.waitKey(1)

#init subscriber node
rospy.init_node(subscriberNodename, anonymous = True)
rospy.Subscriber(topicName, Image, callbackFunction)
rospy.spin()#runs code in inf loop

cv2.destroyAllWindows()



