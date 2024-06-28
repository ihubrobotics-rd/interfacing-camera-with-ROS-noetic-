#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

publisherNodeName = 'camera_sensor_publisher'
topicName = 'video_topic'

def main():
    rospy.init_node(publisherNodeName, anonymous=True)
    publisher = rospy.Publisher(topicName, Image, queue_size=10)
    rate = rospy.Rate(30)

    videoCaptureObject = cv2.VideoCapture(0)

    if not videoCaptureObject.isOpened():
        rospy.logerr("Error: Could not open video device.")
        return

    bridgeObject = CvBridge()

    while not rospy.is_shutdown():
        returnValue, capturedFrame = videoCaptureObject.read()

        if returnValue:
            rospy.loginfo('Video frame captured and published')
            imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding="bgr8")
            publisher.publish(imageToTransmit)

        rate.sleep()

    videoCaptureObject.release()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

