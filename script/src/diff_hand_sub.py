#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp

# Import ROS messages for robot control
from geometry_msgs.msg import Twist

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'
robotCmdTopic = '/cmd_vel'  # Adjust this based on your robot's topic

# Initialize Mediapipe Hand module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# ROS publisher for robot movement commands
robot_publisher = rospy.Publisher(robotCmdTopic, Twist, queue_size=10)

# Helper function to count fingers
def count_fingers(hand_landmarks):
    finger_tips = [8, 12, 16, 20]
    count = 0
    
    for tip in finger_tips:
        # Tip landmark index and its corresponding MCP (Metacarpophalangeal) joint
        tip_y = hand_landmarks.landmark[tip].y
        base_y = hand_landmarks.landmark[tip - 2].y
        if tip_y < base_y:  # If tip is above the base, finger is up
            count += 1
            
    return count

def callbackFunction(message):
    bridgeObject = CvBridge()
    rospy.loginfo("Received a video message/frame")
    capturedFrame = bridgeObject.imgmsg_to_cv2(message)

    # Convert the BGR image to RGB before processing with Mediapipe
    capturedFrame_rgb = cv2.cvtColor(capturedFrame, cv2.COLOR_BGR2RGB)
    
    # Process the frame and detect hands
    results = hands.process(capturedFrame_rgb)

    if results.multi_hand_landmarks:
        capturedFrame_writable = np.copy(capturedFrame)
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw landmarks on the image
            mp_drawing.draw_landmarks(capturedFrame_writable, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Count fingers
            finger_count = count_fingers(hand_landmarks)
            rospy.loginfo(f"Finger count: {finger_count}")
            
            # Control the robot based on finger count
            if finger_count == 1:
                rospy.loginfo("Moving robot forward")
                move_robot_forward()
            elif finger_count == 2:
                rospy.loginfo("Moving robot backward or stopping")
                move_robot_backward()
            elif finger_count == 3:
                rospy.loginfo("Moving robot left side")
                move_robot_left()
            elif finger_count == 4:
                rospy.loginfo("Moving robot right side")
                move_robot_right()
            else:
                rospy.loginfo("No movement command for this finger count")
            
            # Display the count on the image
            cv2.putText(capturedFrame_writable, f"Fingers: {finger_count}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display the frame with annotations
        cv2.imshow("Hand Detection", capturedFrame_writable)
    else:
        cv2.imshow("Hand Detection", capturedFrame)
    
    cv2.waitKey(1)

def move_robot_forward():
    # Publish forward movement command
    twist_msg = Twist()
    twist_msg.linear.x = 0.5  # Adjust speed as needed
    robot_publisher.publish(twist_msg)

def move_robot_backward():
    # Publish backward movement command
    twist_msg = Twist()
    twist_msg.linear.x = -0.5  # Negative value for backward
    robot_publisher.publish(twist_msg)

def move_robot_left():
    # Publish left side movement command (adjust angular.z as needed)
    twist_msg = Twist()
    twist_msg.angular.z = 0.5  # Adjust angular velocity for left turn
    robot_publisher.publish(twist_msg)

def move_robot_right():
    # Publish right side movement command (adjust angular.z as needed)
    twist_msg = Twist()
    twist_msg.angular.z = -0.5  # Adjust angular velocity for right turn
    robot_publisher.publish(twist_msg)

def main():
    rospy.init_node(subscriberNodeName, anonymous=True)
    rospy.Subscriber(topicName, Image, callbackFunction)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

