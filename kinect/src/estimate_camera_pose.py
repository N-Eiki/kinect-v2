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


class EstimatePose(SingleImageSubscriber):
    def rgb_callback(self, rgb):
        rgb = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        self.images = [rgb]

    def plot_pose(self, xyz, Vx, Vy, Vz, elev=105, azim=165):
        fig = plt.figure(figsize=(4,3))
        ax = Axes3D(fig)
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlim(-2, 2); ax.set_ylim(-2, 2); ax.set_zlim(-2, 2)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")

        x, y, z = xyz
        ux, vx, wx = Vx
        uy, vy, wy = Vy
        uz, vz, wz = Vz

        # draw marker
        ax.scatter(0, 0, 0, color="k")
        ax.quiver(0, 0, 0, 1, 0, 0, length=1, color="r")
        ax.quiver(0, 0, 0, 0, 1, 0, length=1, color="g")
        ax.quiver(0, 0, 0, 0, 0, 1, length=1, color="b")
        ax.plot([-1,1,1,-1,-1], [-1,-1,1,1,-1], [0,0,0,0,0], color="k", linestyle=":")

        # draw camera
        ax.quiver(x, y, z, ux, vx, wx, length=0.5, color="r")
        ax.quiver(x, y, z, uy, vy, wy, length=0.5, color="g")
        ax.quiver(x, y, z, uz, vz, wz, length=0.5, color="b")
        
        fig.canvas.draw()
        frame = np.array(fig.canvas.renderer.buffer_rgba())
        return frame

    def est_pose(self):
        while len(self.images)==0:
            time.sleep(0.01)

        mtx = np.array([1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0]).reshape(3,3)
        dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        marker_length = 0.1
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        while True:
            frame = self.images[0]
            cv2.imshow("image", frame)
            cv2.waitKey(1)
            # frame = frame[...,::-1].astype(np.uint8)  # BGR2RGB
            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)
            
            if len(corners) == 0:
                continue
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)

            R = cv2.Rodrigues(rvec)[0]  # 回転ベクトル -> 回転行列
            R_T = R.T
            T = tvec[0].T

            xyz = np.dot(R_T, - T).squeeze()
            # XYZ.append(xyz)

            rpy = np.deg2rad(cv2.RQDecomp3x3(R_T)[0])
            # RPY.append(rpy)
            vx = np.dot(R_T, np.array([1,0,0]))
            vy = np.dot(R_T, np.array([0,1,0]))
            vz = np.dot(R_T, np.array([0,0,1]))
            # V_x.append(np.dot(R_T, np.array([1,0,0])))
            # V_y.append(np.dot(R_T, np.array([0,1,0])))
            # V_z.append(np.dot(R_T, np.array([0,0,1])))
            

            pose1 = self.plot_pose(xyz, vx, vy, vz, elev=105, azim=270)
            pose2 = self.plot_pose(xyz, vx, vy, vz, elev=165, azim=270)
            # ---- 描画
            cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0,255,255))
            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, marker_length/2)
            cv2.imshow('frame', frame)
            cv2.imshow('front', pose1)
            cv2.imshow('front_forward', pose2)
            print('.............')
            print(f"tvec : {xyz}")
            print(f'rot : {R}')
            np.save('/root/catkin_ws/src/kinect/pose/tvec.npy', np.array(xyz))
            np.save('/root/catkin_ws/src/kinect/pose/rot.npy', np.array(R))
            
            cv2.waitKey(1)




if __name__=="__main__":
    rospy.init_node('est_pos')
    image_topic = "/kinect2/hd/image_color_rect"
    sub = EstimatePose(image_topic)
    sub.est_pose()
    