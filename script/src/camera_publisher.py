#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

publisherNodeName = 'camera_sensor_publisher'
topicName = 'video_topic'

rospy.init_node(publisherNodeName, anonymous=True)
publisher = rospy.Publisher(topicName, Image, queue_size=60)
rate = rospy.Rate(30)

videoCaptureObject = cv2.VideoCapture('/dev/video2')
bridgeObject = CvBridge()

while not rospy.is_shutdown():
    returnValue, capturedFrame = videoCaptureObject.read()
    
    if returnValue == True:
        rospy.loginfo('Video frame captured and published')
        
        imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame)
        publisher.publish(imageToTransmit)
    
    rate.sleep()


