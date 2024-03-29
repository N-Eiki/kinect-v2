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


def get_pointcloud(color_img, depth_img, camera_intrinsics):

    # Get depth image size
    im_h = depth_img.shape[0]
    im_w = depth_img.shape[1]

    # Project depth into 3D point cloud in camera coordinates
    pix_x,pix_y = np.meshgrid(np.linspace(0,im_w-1,im_w), np.linspace(0,im_h-1,im_h))
    cam_pts_x = np.multiply(pix_x-camera_intrinsics[0][2],depth_img/camera_intrinsics[0][0])
    cam_pts_y = np.multiply(pix_y-camera_intrinsics[1][2],depth_img/camera_intrinsics[1][1])
    cam_pts_z = depth_img.copy()
    cam_pts_x.shape = (im_h*im_w,1)
    cam_pts_y.shape = (im_h*im_w,1)
    cam_pts_z.shape = (im_h*im_w,1)

    # Reshape image into colors for 3D point cloud
    rgb_pts_r = color_img[:,:,0]
    rgb_pts_g = color_img[:,:,1]
    rgb_pts_b = color_img[:,:,2]
    rgb_pts_r.shape = (im_h*im_w,1)
    rgb_pts_g.shape = (im_h*im_w,1)
    rgb_pts_b.shape = (im_h*im_w,1)

    cam_pts = np.concatenate((cam_pts_x, cam_pts_y, cam_pts_z), axis=1)
    rgb_pts = np.concatenate((rgb_pts_r, rgb_pts_g, rgb_pts_b), axis=1)

    return cam_pts, rgb_pts


def get_heightmap(color_img, depth_img, cam_intrinsics, cam_pose, workspace_limits, heightmap_resolution):

    # Compute heightmap size
    heightmap_size = np.round(((workspace_limits[1][1] - workspace_limits[1][0])/heightmap_resolution, (workspace_limits[0][1] - workspace_limits[0][0])/heightmap_resolution)).astype(int)

    # Get 3D point cloud from RGB-D images
    surface_pts, color_pts = get_pointcloud(color_img, depth_img, cam_intrinsics)

    # Transform 3D point cloud from camera coordinates to robot coordinates
    surface_pts = np.transpose(np.dot(cam_pose[0:3,0:3],np.transpose(surface_pts)) + np.tile(cam_pose[0:3,3:],(1,surface_pts.shape[0])))

    # Sort surface points by z value
    sort_z_ind = np.argsort(surface_pts[:,2])
    surface_pts = surface_pts[sort_z_ind]
    color_pts = color_pts[sort_z_ind]

    # Filter out surface points outside heightmap boundaries

    heightmap_valid_ind = np.logical_and(np.logical_and(np.logical_and(np.logical_and(surface_pts[:,0] >= workspace_limits[0][0], surface_pts[:,0] < workspace_limits[0][1]), surface_pts[:,1] >= workspace_limits[1][0]), surface_pts[:,1] < workspace_limits[1][1]), surface_pts[:,2] < workspace_limits[2][1])
    surface_pts = surface_pts[heightmap_valid_ind]
    color_pts = color_pts[heightmap_valid_ind]

    # Create orthographic top-down-view RGB-D heightmaps
    color_heightmap_r = np.zeros((heightmap_size[0], heightmap_size[1], 1), dtype=np.uint8)
    color_heightmap_g = np.zeros((heightmap_size[0], heightmap_size[1], 1), dtype=np.uint8)
    color_heightmap_b = np.zeros((heightmap_size[0], heightmap_size[1], 1), dtype=np.uint8)
    depth_heightmap = np.zeros(heightmap_size)
    heightmap_pix_x = np.floor((surface_pts[:,0] - workspace_limits[0][0])/heightmap_resolution).astype(int)
    heightmap_pix_y = np.floor((surface_pts[:,1] - workspace_limits[1][0])/heightmap_resolution).astype(int)
    color_heightmap_r[heightmap_pix_y,heightmap_pix_x] = color_pts[:,[0]]
    color_heightmap_g[heightmap_pix_y,heightmap_pix_x] = color_pts[:,[1]]
    color_heightmap_b[heightmap_pix_y,heightmap_pix_x] = color_pts[:,[2]]
    color_heightmap = np.concatenate((color_heightmap_r, color_heightmap_g, color_heightmap_b), axis=2)
    depth_heightmap[heightmap_pix_y,heightmap_pix_x] = surface_pts[:,2]
    
    z_bottom = workspace_limits[2][0]
    depth_heightmap = depth_heightmap - z_bottom
    depth_heightmap[depth_heightmap < 0] = 0
    depth_heightmap[depth_heightmap == -z_bottom] = np.nan
    return color_heightmap, depth_heightmap


if __name__== '__main__':
    def get_args():
        import argparse
        """ Get arguments for individual tb3 deployment. """
        parser = argparse.ArgumentParser(
            description=""
        )

        # Required arguments
        parser.add_argument(
            '--image_topic', default="/hsrb/head_rgbd_sensor/rgb/image_raw", 
            choices=[
            "/hsrb/head_rgbd_sensor/rgb/image_raw",
            "/hsrb/head_r_stereo_camera/image_raw",
            "/hsrb/hand_camera/image_raw",
            "/hsrb/head_rgbd_sensor/depth_registered/image_raw"
            ]
        )
        return parser.parse_args()
    
    rospy.init_node('get_single_transformed_image')
    args = get_args()
    topic_dir = args.image_topic.split('/')[-3]
    # import ipdb;ipdb.set_trace()
    
    camera_matrix = np.load(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'hsr_images', 'params', topic_dir, 'calibration', 'mtx.npy'))
    
    R = np.load(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'hsr_images', 'params', topic_dir, 'camera_pose', 'rotation.npy'))
    t = np.load(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'hsr_images', 'params', topic_dir, 'camera_pose', 'tvec.npy'))
    cam_trans = np.eye(4)
    cam_trans[0:3,3:] = t/100
    cam_rotm = np.eye(4)
    cam_rotm[:3,:3] = R
    cam_pose = np.dot(cam_trans, cam_rotm)
    # cam_pose = np.load(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'hsr_images', 'params', topic_dir, 'camera_pose', 'cam_pose.npy'))
    # workspace_limits = np.asarray([[-0.5, 0.5], [-0.5, 0.5], [-1,10]])
    workspace_limits = np.asarray([[-0.35, 0.35], [-0.05, .7], [-10,10]])

    heightmap_resolution = 0.5/1024#224
    
    rgb_image = cv2.imread(os.path.join(os.path.dirname(os.path.dirname((os.path.dirname(__file__)))), 'hsr_images', 'images', 'rgb.png'))
    depth_image = np.load(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'hsr_images', 'images', 'depth.npy')) / 1000.

    ret_rgb, ret_depth = get_heightmap(rgb_image, depth_image, camera_matrix, cam_pose, workspace_limits, heightmap_resolution)
    # plt_rgb = np.concatenate([rgb_image, ret_rgb], axis=1)
    # plt_depth = np.concatenate([depth_image, ret_depth], axis=1)
    
    
    plt.imshow(ret_rgb);plt.show()
    # plt.imshow(plt_depth);plt.show()