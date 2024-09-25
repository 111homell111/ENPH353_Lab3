#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Initialize the CVBridge
bridge = CvBridge()

cx = 400

def image_callback(ros_image):
    global cx
    # Convert the ROS Image message to a format OpenCV understands
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
    except CvBridgeError as e:
        print("CVBridgeError")
        rospy.logerr(f"CVBridge Error: {e}")
        return

    #height, width, _ = frame.shape
    #print(height, width)

    frame, skipped, cx = analyze_image(frame)
    print(f"CX: {cx}")

    if not skipped:
        move([0.8, 0, 0], [0, 0, -(cx-400)/100])
    else:
        move([0.5, 0, 0], [0, 0, -((cx-400)/100) *1.5])


    #print("Live Video Feed")s
    cv2.imshow("Live Video Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('Quit')

def analyze_image(frame):
    global cx

    skipped = False
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (3, 3), 0)

    _, mask = cv2.threshold(gray_frame, 130, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        cv2.drawContours(gray_frame, max(contours, key=cv2.contourArea), -1, (0, 255, 0), 2)
        M = cv2.moments(max(contours, key=cv2.contourArea))
        cx = int(M["m10"]/M["m00"])
        #cy = int(M["m01"]/M["m00"])
    else:
        print("skipped")
        skipped=True
    
    return mask, skipped, cx

def move(linear_velocities = [0,0,0], angular_velocities = [0,0,0]):
    # Create a publisher to publish Twist messages to /cmd_vel
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Create a Twist message
    vel_msg = Twist()

    # Set linear velocity (forward motion)
    vel_msg.linear.x = linear_velocities[0]  # Adjust the speed (m/s)
    vel_msg.linear.y = linear_velocities[1]
    vel_msg.linear.z = linear_velocities[2]

    # Set angular velocity (rotation)
    vel_msg.angular.x = angular_velocities[0]
    vel_msg.angular.y = angular_velocities[1]
    vel_msg.angular.z = angular_velocities[2]  # Adjust the rotational speed (rad/s)
    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        cx = 400
        # Initialize the ROS node
        rospy.init_node('robot_controller_with_video', anonymous=True)

        # Subscribe to the image topic (make sure your camera is publishing on this topic)
        image_topic = "/robot/camera1/image_raw"  # Adjust the topic name as per your setup
        rospy.Subscriber(image_topic, Image, image_callback)
        # Move the robot
        move([0.1, 0, 0], [0, 0, 0])

        # Spin until node is shut down
        rospy.spin()


    except rospy.ROSInterruptException:
        pass

    finally:
        # Close OpenCV windows when done
        cv2.destroyAllWindows()