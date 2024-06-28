#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

# Load Haar Cascade for face detection
faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def callbackFunction(message):
    try:
        bridgeObject = CvBridge()
        # Convert ROS Image message to OpenCV image
        cv_image = bridgeObject.imgmsg_to_cv2(message, desired_encoding="bgr8")
        
        # Perform face detection
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        
        # Draw rectangles around detected faces and display message
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            rospy.loginfo("Face detected!")
        else:
            rospy.loginfo("No face detected.")

        # Display the image
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)  # Refresh display (1 ms)
    
    except Exception as e:
        print(e)

rospy.init_node(subscriberNodeName, anonymous=True)
rospy.Subscriber(topicName, Image, callbackFunction)
rospy.spin()

cv2.destroyAllWindows()

