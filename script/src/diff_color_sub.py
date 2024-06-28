#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Import ROS messages for robot control
from geometry_msgs.msg import Twist

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'
robotCmdTopic = '/cmd_vel'  # Adjust this based on your robot's topic

# Define color ranges for detection (adjust as needed)
color_ranges = {
    'red': ((0, 100, 100), (10, 255, 255)),
    'yellow': ((20, 100, 100), (30, 255, 255)),
    'blue': ((110, 100, 100), (130, 255, 255))
}

# ROS publisher for robot movement commands
robot_publisher = rospy.Publisher(robotCmdTopic, Twist, queue_size=10)

# Helper function to detect colors in an image
def detect_color(image, color):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_range, upper_range = color_ranges[color]
    mask = cv2.inRange(hsv, lower_range, upper_range)
    return mask

def move_robot(command):
    twist_msg = Twist()
    if command == "forward":
        twist_msg.linear.x = 0.5  # Adjust speed as needed
    elif command == "backward":
        twist_msg.linear.x = -0.5  # Negative value for backward
    elif command == "left":
        twist_msg.angular.z = 0.5  # Adjust angular velocity for left turn
    elif command == "right":
        twist_msg.angular.z = -0.5  # Adjust angular velocity for right turn
    elif command == "stop":
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
    robot_publisher.publish(twist_msg)

def callbackFunction(message):
    bridgeObject = CvBridge()
    rospy.loginfo("Received a video message/frame")
    capturedFrame = bridgeObject.imgmsg_to_cv2(message, desired_encoding="bgr8")

    # Detect colors in the received frame
    red_mask = detect_color(capturedFrame, 'red')
    yellow_mask = detect_color(capturedFrame, 'yellow')
    blue_mask = detect_color(capturedFrame, 'blue')

    if cv2.countNonZero(yellow_mask) > 0:
        rospy.loginfo('Yellow color detected: Moving robot forward')
        move_robot("forward")
    elif cv2.countNonZero(red_mask) > 0:
        rospy.loginfo('Red color detected: Moving robot right')
        move_robot("right")
    elif cv2.countNonZero(blue_mask) > 0:
        rospy.loginfo('Blue color detected: Moving robot left')
        move_robot("left")
    else:
        rospy.loginfo('No color detected: Stopping robot')
        move_robot("stop")

    # Display the frame
    cv2.imshow("camera", capturedFrame)
    cv2.waitKey(1)

def main():
    rospy.init_node(subscriberNodeName, anonymous=True)
    rospy.Subscriber(topicName, Image, callbackFunction)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

