#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Initialize the CVBridge
bridge = CvBridge()

#Global variable representing the x-axis center of mass
cx = 400


##
# \brief Callback function for processing image data from the camera.
# \param ros_image The image data received from the camera in ROS format.
def image_callback(ros_image):
    global cx
    # Convert the ROS Image message an OpenCV format 
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8') #Frame is 800x800
    except CvBridgeError as e:
        print("CVBridgeError")
        rospy.logerr(f"CVBridge Error: {e}")
        return

    #Locates path contour (could be optimized)
    frame, skipped, cx = analyze_image(frame)
    print(f"CX: {cx}")

    #If contour was found
    if not skipped:
        move([0.8, 0, 0], [0, 0, -(cx-400)/100])
    else: #If contour was not found
        move([0.5, 0, 0], [0, 0, -((cx-400)/100) *1.5])


    #print("Live Video Feed")s
    cv2.imshow("Live Video Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('Quit')


##
# \brief Analyzes the given frame to detect contours and compute the center of mass.
# \param frame The image to analyze (800x800).
# \return A tuple containing the processed mask, whether a conotur was located, and the x-coordinate of the center of mass (cx).
def analyze_image(frame):
    global cx
    skipped = False

    #Change image to grayscale and blur to remove noise
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (3, 3), 0)

    #Threshold to look for dark lines
    _, mask = cv2.threshold(gray_frame, 130, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours: #If contour found
        #Draw the largest contour
        cv2.drawContours(gray_frame, max(contours, key=cv2.contourArea), -1, (0, 255, 0), 2)
        #Calculate moments
        M = cv2.moments(max(contours, key=cv2.contourArea))
        #Find center of mass (x)
        cx = int(M["m10"]/M["m00"])
        #cy = int(M["m01"]/M["m00"])
    else:
        print("skipped")
        skipped=True
    
    return mask, skipped, cx


##
# \brief Publishes velocity commands to move the robot.
# \param linear_velocities A list of three values [x, y, z] representing linear velocity in m/s.
# \param angular_velocities A list of three values [x, y, z] representing angular velocity in rad/s.
def move(linear_velocities = [0,0,0], angular_velocities = [0,0,0]):
    # Create a publisher to publish Twist messages to /cmd_vel
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Create a Twist message
    vel_msg = Twist()

    # Set linear velocity (forward motion)
    vel_msg.linear.x = linear_velocities[0]  
    vel_msg.linear.y = linear_velocities[1]
    vel_msg.linear.z = linear_velocities[2]

    # Set angular velocity (rotation)
    vel_msg.angular.x = angular_velocities[0]
    vel_msg.angular.y = angular_velocities[1]
    vel_msg.angular.z = angular_velocities[2]  
    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('robot_controller_with_video', anonymous=True)

        # Subscribe to the image topic (make sure your camera is publishing on this topic)
        image_topic = "/robot/camera1/image_raw"  # Adjust the topic name as per your setup
        rospy.Subscriber(image_topic, Image, image_callback)

        # Spin until node is shut down
        rospy.spin()


    except rospy.ROSInterruptException:
        pass

    finally:
        # Close OpenCV windows when done
        cv2.destroyAllWindows()