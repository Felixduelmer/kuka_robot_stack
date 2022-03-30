#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import os
import cv2
import ros_numpy


class ImagePublisher(object):

  def __init__(self):
    self.node_rate = 0.5
    self.image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
    self.path = '/home/robotics-verse/projects/felix/DataSet/dummy_image_stream'

  def run(self):
    rospy.loginfo("Starting run function")
    loop = rospy.Rate(self.node_rate)
    for image in sorted([num.split('.')[0] for num in os.listdir(self.path)]):
        if rospy.is_shutdown():
          break
        image = str(image) + '.png'
        rospy.loginfo("publishing image:" + image)
        cv_image = cv2.imread(self.path + '/' + image)
        image_message = ros_numpy.msgify(Image, cv_image, encoding="rgb8")
        self.image_pub.publish(image_message)   
        loop.sleep()

if __name__ == '__main__':
  rospy.loginfo("starting")
  rospy.init_node('image_node')
  ImagePublisher().run()
