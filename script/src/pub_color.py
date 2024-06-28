#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

publisherNodeName = 'camera_sensor_publisher'
topicName = 'video_topic'

# Load Haar Cascade for face detection (example, replace with color detection logic)
# For color detection, you won't need the cascade classifier, but instead, use color thresholding
# Define color thresholds in HSV format
# Example thresholds for Red, Yellow, Blue in HSV:
color_ranges = {
    'red': ((0, 100, 100), (10, 255, 255)),
    'yellow': ((20, 100, 100), (30, 255, 255)),
    'blue': ((110, 100, 100), (130, 255, 255))
}

# Helper function to detect colors in an image
def detect_color(image, color):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_range, upper_range = color_ranges[color]
    mask = cv2.inRange(hsv, lower_range, upper_range)
    return mask

def main():
    rospy.init_node(publisherNodeName, anonymous=True)
    publisher = rospy.Publisher(topicName, Image, queue_size=10)
    rate = rospy.Rate(30)

    # Initialize VideoCapture with your USB camera device (adjust X accordingly)
    videoCaptureObject = cv2.VideoCapture('/dev/video2')  # Replace with the correct camera index if not 0

    # Check if the camera is opened successfully
    if not videoCaptureObject.isOpened():
        rospy.logerr("Error: Could not open video device.")
        return

    bridgeObject = CvBridge()

    while not rospy.is_shutdown():
        returnValue, capturedFrame = videoCaptureObject.read()
        
        if returnValue:
            rospy.loginfo('Video frame captured and published')
            
            # Example: detect red color in the frame
            red_mask = detect_color(capturedFrame, 'red')
            
            # Example: detect yellow color in the frame
            yellow_mask = detect_color(capturedFrame, 'yellow')
            
            # Example: detect blue color in the frame
            blue_mask = detect_color(capturedFrame, 'blue')

            # Publish the original image (you can also publish the masks as needed)
            imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding="bgr8")
            publisher.publish(imageToTransmit)
        
        rate.sleep()

    videoCaptureObject.release()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

