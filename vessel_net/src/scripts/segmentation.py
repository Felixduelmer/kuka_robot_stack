#!/usr/bin/env python3
# Import ROS libraries and messages

from utils.util import json_file_to_pyobj
from models import get_model
import rospy
import ros_numpy
from sensor_msgs.msg import Image
import collections
import numpy as np
from vessel_net.msg import array_image 
import torch
import gc
from ros_igtl_bridge.msg import igtlimage


class SegmentationNetwork:
    def __init__(self):
        #self.json_filename = rospy.get_param("SegNode/path_for_config")
        self.json_opts = json_file_to_pyobj('/home/robotics-verse/projects/felix/vessel-tracking/image_seg_network/configs/config_vesnet_inference.json')
        self.polyaxon_output_path = None
        self.model = get_model(self.json_opts.model, self.polyaxon_output_path)
        self.model.init_hidden()
        self.buffer_size = 10
        self.seg_img_pub = rospy.Publisher("IGTL_IMAGE_OUT", igtlimage, queue_size=10)

        #store images
        self.input_images_buffer = collections.deque([np.zeros((2, 320, 320))]*self.buffer_size, maxlen=self.buffer_size)
        rospy.Subscriber("images/preprocessed", array_image, self.inference)

    def inference(self, arr_im_msg):
        self.model.init_hidden()    
        self.input_images_buffer.append(np.concatenate(
            [ros_numpy.numpify(arr_im_msg.data[0])[None, :, :], ros_numpy.numpify(arr_im_msg.data[1])[None, :, :]], axis=0))
        self.model.set_input(torch.from_numpy(np.array([self.input_images_buffer[i] for i in range(-5, 0)])))
        res = self.model.inference().cpu().numpy()
        res = np.uint8(np.squeeze(res[-1]))
        res_msg = igtlimage()
        print(res.shape)
        res_msg.x_steps = 320
        res_msg.y_steps = 320
        res_msg.z_steps = 1
        res_msg.data = list(res.flatten())
        # res_msg = ros_numpy.msgify(Image,res, encoding="mono8")
        self.seg_img_pub.publish(res_msg)
        





def main():
    rospy.init_node('SegNode', anonymous=True)
    SegmentationNetwork()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()