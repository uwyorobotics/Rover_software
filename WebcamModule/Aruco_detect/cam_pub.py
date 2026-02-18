#!/usr/bin/env python3
#telling python that this is a python source file

import rospy

from sensor_msgs.msg import Image

#cv _bridge interfaced open_cv data to ros data and back
from cv_bridge import CvBridge
import cv2

#name our publisher node;
publisherNodeName = 'cam_pub'

#name the topic
topicName = 'videoTopic'

#initialize the node
rospy.init_node(publisherNodeName, anonymous = True)

#create the publisher object
publisher = rospy.Publisher(topicName,Image,queue_size = 60)

#define the rate
rate = rospy.Rate(30)

#create the video capture object
videoCaptureObject = cv2.VideoCapture(0)

#convert the cv images to ros image Mesages
bridgeObject = CvBridge()

#loop: capture the images and transmit them to the topic

while not rospy.is_shutdown():
	returnVal, capturedFrame = videoCaptureObject.read()
	#if this is ok, transmit onto the topic
	if  returnVal == True:
		#bridge
		rospy.loginfo('frame captured and published')
		ImageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame)
		#publish
		publisher.publish(ImageToTransmit)
	rate.sleep() #sleep to acheve the desired rate
	
#good job!
