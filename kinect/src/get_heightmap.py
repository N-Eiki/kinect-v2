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
from mpl_toolkits.mplot3d import axes3d, Axes3D

from kinect.src.subscribe_color import SingleImageSubscriber
from kinect.src.heightmap_utils import get_heightmap

class ImageSub(SingleImageSubscriber):
    def rgb_callback(self, rgb):
        rgb = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        if self.all:
            self.images.append(rgb)
        else:
            self.images = [rgb]
        
    def set_save_flg(self, flg):
        self.all = flg
        
    def depth_callback(self, depth):
        depth = self.bridge.imgmsg_to_cv2(depth, '32FC1')
        depth /= 1000.
        # self.images.append(depth)
        self.images = [depth]
            

def get_workspace_area(rgb):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, _ = cv2.aruco.detectMarkers(rgb, aruco_dict)
    ret = rgb.copy()
    corners[2], corners[3] = corners[3], corners[2]
    points = np.array([corner[0][0] for corner in corners]).astype(int)
    # points[2], points[3] = points[3], points[2]

    cv2.fillConvexPoly(ret, points, color=(255, 0, 0))    
    return ret




if __name__=="__main__":
    rospy.init_node('est_pos')
    color_topic = "/kinect2/hd/image_color_rect"
    depth_topic = "/kinect2/hd/image_depth_rect"
    color_sub = ImageSub(color_topic)
    depth_sub = ImageSub(depth_topic)

    mtx = np.array([1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0]).reshape(3,3)
    dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    R = np.load("/root/catkin_ws/src/kinect/pose/rot.npy")
    t = np.load("/root/catkin_ws/src/kinect/pose/tvec.npy")

    # ロール、ピッチ、ヨーの角度を90度ずつ設定（ラジアン単位）
    roll_angle = np.radians(90)
    pitch_angle = np.radians(90)
    yaw_angle = np.radians(90)

    # ロール回転行列を計算
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll_angle), -np.sin(roll_angle)],
        [0, np.sin(roll_angle), np.cos(roll_angle)]
    ])

    # ピッチ回転行列を計算
    R_pitch = np.array([
        [np.cos(pitch_angle), 0, np.sin(pitch_angle)],
        [0, 1, 0],
        [-np.sin(pitch_angle), 0, np.cos(pitch_angle)]
    ])

    # ヨー回転行列を計算
    R_yaw = np.array([
        [np.cos(yaw_angle), -np.sin(yaw_angle), 0],
        [np.sin(yaw_angle), np.cos(yaw_angle), 0],
        [0, 0, 1]
    ])


    R = np.dot(R_roll, R)
    # t[0] = 0
    # t = np.array([[0, 0, 0]]).reshape(3, 1)
    cam_trans = np.eye(4)
    cam_trans[0:3,3:] = t.reshape(-1, 1)
    cam_rotm = np.eye(4)
    cam_rotm[:3,:3] = R
    cam_pose = np.dot(cam_trans, cam_rotm)
    workspace_limits = np.asarray([[-1.5, 1.5], [-2, 1], [-1, 3]])
    heightmap_resolution = 0.5/224
    xy_area = np.load('/root/catkin_ws/src/kinect/pose/workspace_05_224.npy')
    xmin, xmax, ymin, ymax = xy_area
    while len(color_sub.images)==0 or len(depth_sub.images)==0:
        time.sleep(0.01)
    
    while True:
    # if True:
        rgb_image = color_sub.images[0]
        depth_image = depth_sub.images[0]
        depth_image[depth_image>8] = 0
        top_down_rgb, ret_depth = get_heightmap(rgb_image, depth_image, mtx, cam_pose, workspace_limits, heightmap_resolution)       
        plt.imshow(top_down_rgb[xmin:xmax, ymin:ymax])
        plt.pause(1/30)
