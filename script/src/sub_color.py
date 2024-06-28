#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

# Define color ranges for detection (adjust as needed)
color_ranges = {
    'red': ((0, 100, 100), (10, 255, 255)),
    'yellow': ((20, 100, 100), (30, 255, 255)),
    'blue': ((110, 100, 100), (130, 255, 255))
}

def detect_color(image, color):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_range, upper_range = color_ranges[color]
    mask = cv2.inRange(hsv, lower_range, upper_range)
    return mask

def check_color_distance(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        min_area_threshold = 5000  # Example threshold, adjust as needed
        if area > min_area_threshold:
            return True
    return False

def callbackFunction(message):
    bridgeObject = CvBridge()
    rospy.loginfo("--------")
    capturedFrame = bridgeObject.imgmsg_to_cv2(message)

    # Example: detect red color in the received frame
    red_mask = detect_color(capturedFrame, 'red')
    if cv2.countNonZero(red_mask) > 0 and check_color_distance(red_mask):
        rospy.loginfo('Red color detected')
 
    # Example: detect yellow color in the received frame
    yellow_mask = detect_color(capturedFrame, 'yellow')
    if cv2.countNonZero(yellow_mask) > 0 and check_color_distance(yellow_mask):
        rospy.loginfo('Yellow color detected')
    

    # Example: detect blue color in the received frame
    blue_mask = detect_color(capturedFrame, 'blue')
    if cv2.countNonZero(blue_mask) > 0 and check_color_distance(blue_mask):
        rospy.loginfo('Blue color detected ')
    
    cv2.imshow("camera", capturedFrame)
    cv2.waitKey(1)

def main():
    rospy.init_node(subscriberNodeName, anonymous=True)
    rospy.Subscriber(topicName, Image, callbackFunction)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

