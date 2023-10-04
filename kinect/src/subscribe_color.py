#!/usr/bin/env python3

import math
import numpy as np

import os
import sys
import time
import  matplotlib.pyplot as plt

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import message_filters

from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
# from hsr_images.srv import RGBD, RGBDResponse
# from hsr_images.script.subscribe_images import SingleImageSubscriber

class SingleImageSubscriber:
    def __init__(self, topic_name, all=False):
        self.all = all #save all images?
        if "compressed" in topic_name:
            image_type = CompressedImage
        else:
            image_type = Image

        if "depth" in topic_name:
            depth_sub = rospy.Subscriber(topic_name, image_type, self.depth_callback)
        else:
            rgb_sub = rospy.Subscriber(topic_name, image_type, self.rgb_callback)
        self.images = []
        self.bridge = CvBridge()

    def rgb_callback(self, rgb):
        rgb = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        self.images.append(rgb)
        print(rgb.shape)
        cv2.imshow('image', rgb)
        cv2.imwrite('rgb.png', rgb)
        cv2.waitKey(1)
        

    def depth_callback(self, depth):
        depth = self.bridge.imgmsg_to_cv2(depth, '32FC1')
        # self.images.append(depth)
        print(depth.shape)
        plt.imshow(depth)
        np.save('depth.npy', depth)
        plt.pause(.01)
        
def calibration(sub, topic_dir):
    pattern_size = (10, 7)
    object_points = []
    image_points = []
    obuject_points_2d = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
    obuject_points_2d[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    time.sleep(10)
    while len(sub.images)==0:
        time.sleep(0.01)
    
    print('start')
    from hsr_head.script.head import Head
    print('moving head')
    head = Head()
    target = {"pan":0.0, "tilt":0.0}
    head.move_head_pose(target, duration=5)
    while len(object_points)<20:

        image = sub.images[-1]
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not ret:
            print('cont')
            continue
        object_points.append(obuject_points_2d)
        image_points.append(corners)
        print(f"done {len(object_points)}")
        time.sleep(2)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, gray.shape[::-1], None, None
    )
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'params', topic_dir, 'calibration')
    os.makedirs(path, exist_ok=True)
    np.save(os.path.join(path, f'mtx_pan{target["pan"]}_tilt{target["tilt"]}.npy'), mtx)
    np.save(os.path.join(path, f'dist_pan{target["pan"]}_tilt{target["tilt"]}.npy'), dist)
    np.save(os.path.join(path, f'rvecs_pan{target["pan"]}_tilt{target["tilt"]}.npy'), rvecs)
    np.save(os.path.join(path, f'tvecs_pan{target["pan"]}_tilt{target["tilt"]}.npy'), tvecs)
    import ipdb;ipdb.set_trace()

if __name__=="__main__":
    rospy.init_node('subimg')
    image_topic_list = [
        "/kinect2/hd/image_color",
        "/kinect2/hd/image_color_rect",
        "/kinect2/hd/image_depth",
        "/kinect2/hd/image_depth_rect"
    ]
    
    image_topic = image_topic_list[1]
    # topic_dir = image_topic.split("/")[1]
    sub = SingleImageSubscriber(image_topic)
    # calibration(sub, topic_dir)
    rospy.spin()
