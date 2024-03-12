#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import CartesianPose

def callback(data):
    # Extract the PoseStamped part of the incoming message
    pose_stamped = data.poseStamped

    # Publish the PoseStamped part
    pub.publish(pose_stamped)

def listener():
    rospy.init_node('pose_publisher', anonymous=True)

    # Subscribe to the topic where the combined message is published
    # Replace 'your_topic_name' with the actual topic name
    rospy.Subscriber('iiwa/state/CartesianPose', CartesianPose, callback)

    # Define the publisher to publish on a new topic
    # You can name the new topic whatever is appropriate
    global pub
    pub = rospy.Publisher('pose_stamped_topic', PoseStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()