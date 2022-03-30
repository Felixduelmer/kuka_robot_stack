#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import os
import cv2
import ros_numpy
import collections
import numpy as np
from math import hypot, pi
from rospy.numpy_msg import numpy_msg
from vessel_net.msg import array_image
from ros_igtl_bridge.msg import igtlimage, igtltransform
import geometry_msgs.msg

class PreProcessing(object):

    def __init__(self):
        self.path = '/home/robotics-verse/projects/felix/DataSet/dummy_image_stream'
        # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
        self.sub_image = rospy.Subscriber("image_topic", Image, self.pre_processing)
        self.image_pub = rospy.Publisher("images/preprocessed", array_image, queue_size=10)
        self.pub_image_us = rospy.Publisher("images/preprocessed/us", Image, queue_size=10)
        self.pub_image_doppler = rospy.Publisher("images/preprocessed/doppler", Image, queue_size=10)
        self.seg_img_pub = rospy.Publisher("IGTL_IMAGE_OUT", igtlimage, queue_size=10)
        self.seg_transform_pub = rospy.Publisher("IGTL_TRANSFORM_OUT", igtltransform, queue_size=10)

        #preprocessing pipeline
        self.buffer_size = 10
        self.minimal_occurence_in_percent = 0.7
        self.max_distance = 30
        self.count = 0
        self.tracking_objects = {}
        self.track_id = 0

        # dummy values for slicer 
        self.transform_msg = igtltransform()
        self.transform_msg.transform = geometry_msgs.msg.Transform()
        self.transform_msg.transform.rotation.w = 0
        self.transform_msg.transform.rotation.x = 0
        self.transform_msg.transform.rotation.y = 1
        self.transform_msg.transform.rotation.z = 0
        self.transform_msg.transform.translation.x = 0
        self.transform_msg.transform.translation.y = 0
        self.transform_msg.transform.translation.z = 0

        self.transform_msg.name = "test_transform"
        print("Init complete")

    def resizer(self, img):
        print(img.shape)
        return cv2.resize(img, (320, 320))


    def pre_processing(self, img_msg):
        # decode img message and transpose
        print("received img message")
        img = ros_numpy.numpify(img_msg)
        # cropping
        img_us = img[136:835, 472:973, :]
        img_doppler = img[136:835, 1004:1505, :]
        # resizing
        img_us = self.resizer(img_us)
        img_doppler = self.resizer(img_doppler)
        hsv = cv2.cvtColor(img_doppler, cv2.COLOR_RGB2HSV)
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(
            hsv, np.array([0, 100, 20]), np.array([180, 255, 255]))
        # Bitwise-AND mask and original image
        img_doppler = cv2.bitwise_and(img_doppler, img_doppler, mask=mask)


        #convert to grayscale
        img_us = np.uint8(cv2.cvtColor(img_us, cv2.COLOR_BGR2GRAY))
        img_doppler = np.uint8(cv2.cvtColor(img_doppler, cv2.COLOR_BGR2GRAY))

        #stabilize doppler
        img_doppler = np.uint8(self.stabilize_doppler(img_doppler))
        img_us_msg = ros_numpy.msgify(Image, img_us, encoding="mono8")
        img_doppler_msg = ros_numpy.msgify(Image, img_doppler, encoding="mono8")
        image_array_msg = array_image()
        image_array_msg.data = [img_us_msg, img_doppler_msg]
        self.image_pub.publish(image_array_msg)
        self.pub_image_doppler.publish(img_doppler_msg)
        self.pub_image_us.publish(img_us_msg)
        res_msg = igtlimage()
        res_msg.x_steps = 320
        res_msg.y_steps = 320
        res_msg.z_steps = 1
        res_msg.x_spacing = 1
        res_msg.y_spacing = 1
        res_msg.z_spacing = 1
        res_msg.name = "test image"
        print(img_us.shape)
        res_msg.data =  list(img_us.flatten())
        # res_msg = ros_numpy.msgify(Image,res, encoding="mono8")
        self.seg_img_pub.publish(res_msg)

        self.seg_transform_pub.publish(self.transform_msg)
        self.transform_msg.transform.translation.x += 1

        

    
    def stabilize_doppler(self, img):
        # image = str(image) + '.png'
        # Point current frame
        contour_properties_cur_frame = []
        img[img > 0] = 255

        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for i, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            cen, rad = cv2.minEnclosingCircle(contours_poly)
            if rad < 5:
                continue
            contour_properties_cur_frame.append(ContourProperties(tuple(int(el) for el in cen), int(rad)))

        tracking_objects_copy = self.tracking_objects.copy()
        contour_properties_cur_frame_copy = contour_properties_cur_frame.copy()

        for object_id, cp2 in tracking_objects_copy.items():
            object_exists = False
            for cp in contour_properties_cur_frame_copy:
                distance = hypot(cp2[0].center[0] - cp.center[0], cp2[0].center[1] - cp.center[1])

                # Update IDs position
                if distance < self.max_distance:
                    if object_exists:
                        if self.tracking_objects[object_id][0].area > cp.area:
                            continue
                        else:
                            contour_properties_cur_frame.append(self.tracking_objects[object_id].popleft())

                    self.tracking_objects[object_id].appendleft(cp)
                    object_exists = True
                    if cp in contour_properties_cur_frame:
                        contour_properties_cur_frame.remove(cp)

            # Pop element if it has not been visible recently or add a dummy value
            if not object_exists:
                if len([el for el in self.tracking_objects[object_id] if
                        el.dummyValue]) > self.buffer_size * self.minimal_occurence_in_percent:
                    self.tracking_objects.pop(object_id)
                else:
                    self.tracking_objects[object_id].appendleft(ContourProperties((0, 0), 0, True))

        # Add new IDs found
        for cp in contour_properties_cur_frame:
            self.tracking_objects[self.track_id] = collections.deque([cp], maxlen=self.buffer_size)
            self.track_id += 1

        annotations = np.zeros(img.shape)

        for object_id, cp in self.tracking_objects.items():
            if len([el for el in self.tracking_objects[object_id] if
                    not el.dummyValue]) < self.buffer_size * self.minimal_occurence_in_percent:
                continue
            center_point = next(el.center for el in cp if not el.dummyValue)
            max_radius = max(el.rad for el in cp if not el.dummyValue)

            cv2.circle(annotations, center_point, max_radius, (255, 255, 255), -1)

        self.count += 1
        return annotations

class ContourProperties:
    def __init__(self, center, radius, dummy_value=False):
        self.center = center
        self.rad = radius
        self.dummyValue = dummy_value
        self.area = pi * (radius ** 2)


if __name__ == '__main__':
  rospy.loginfo("Starting Preprocessing Node")
  rospy.init_node('PreProcessing')
  PreProcessing()
  while not rospy.is_shutdown():
    rospy.spin()
