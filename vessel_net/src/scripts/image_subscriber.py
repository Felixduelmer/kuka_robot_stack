#!/usr/bin/env python3
# Import ROS libraries and messages

import rospy
from sensor_msgs.msg import Image
import ros_numpy
import cv2


# Print "Hello!" to terminal
print("Hello!")

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("preview", img)
    cv2.waitKey(1)


# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)
    np_img = ros_numpy.numpify(img_msg)


    # Show the converted image
    show_image(np_img)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("image_topic", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()