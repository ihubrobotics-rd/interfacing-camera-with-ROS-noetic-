#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

# Initialize Mediapipe Hand module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# Helper function to count fingers
def count_fingers(hand_landmarks):
    finger_tips = [8, 12, 16, 20]  # Indices for tips of the index, middle, ring, and pinky fingers
    count = 0
    
    for tip in finger_tips:
        # Tip landmark index and its corresponding MCP (Metacarpophalangeal) joint
        tip_y = hand_landmarks.landmark[tip].y
        base_y = hand_landmarks.landmark[tip - 2].y
        if tip_y < base_y:  # If tip is above the base, finger is up
            count += 1
    
    # Check thumb separately
    thumb_tip = hand_landmarks.landmark[4]
    thumb_ip = hand_landmarks.landmark[3]
    thumb_mcp = hand_landmarks.landmark[2]
    
    # Thumb is up if tip is above IP and MCP joint in y-axis
    if thumb_tip.x < thumb_ip.x < thumb_mcp.x or thumb_tip.x > thumb_ip.x > thumb_mcp.x:
        count += 1

    return count

def callbackFunction(message):
    bridgeObject = CvBridge()
    rospy.loginfo("Received a video message/frame")
    capturedFrame = bridgeObject.imgmsg_to_cv2(message)
    
    # Convert the frame to a writable format
    capturedFrame_writable = np.copy(capturedFrame)

    # Convert the BGR image to RGB before processing with Mediapipe
    capturedFrame_rgb = cv2.cvtColor(capturedFrame_writable, cv2.COLOR_BGR2RGB)
    
    # Process the frame and detect hands
    results = hands.process(capturedFrame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw landmarks on the image
            mp_drawing.draw_landmarks(capturedFrame_writable, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Count fingers
            finger_count = count_fingers(hand_landmarks)
            rospy.loginfo(f"Finger count: {finger_count}")
            
            # Display the count on the image
            cv2.putText(capturedFrame_writable, f"Fingers: {finger_count}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(capturedFrame_writable, f"Fingers: 0", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Display the frame with annotations
    cv2.imshow("Hand Detection", capturedFrame_writable)
    cv2.waitKey(1)

def main():
    rospy.init_node(subscriberNodeName, anonymous=True)
    rospy.Subscriber(topicName, Image, callbackFunction)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

