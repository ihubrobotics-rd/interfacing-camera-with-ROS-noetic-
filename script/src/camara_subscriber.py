#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

def callbackFunction(message):
    bridgeObject = CvBridge()
    try:
        rospy.loginfo("Received a video message/frame")
        # Convert ROS Image message to OpenCV format
        convertedFrameBackToCV = bridgeObject.imgmsg_to_cv2(message, desired_encoding="passthrough")
        
        # Display the image using OpenCV
        cv2.imshow("camera", convertedFrameBackToCV)
        cv2.waitKey(1)
    
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

def main():
    rospy.init_node(subscriberNodeName, anonymous=True)
    rospy.Subscriber(topicName, Image, callbackFunction)
    rospy.spin()

    # When shutting down, close the OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

