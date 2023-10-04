#!/usr/bin/env python3

import math
import numpy as np
from pyquaternion import Quaternion

import os
import sys
import time

import  matplotlib.pyplot as plt

import cv2
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation

import rospy
import message_filters
from std_msgs.msg import Float32MultiArray

from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from mpl_toolkits.mplot3d import axes3d, Axes3D

# from kinect.src.subscribe_color import SingleImageSubscriber

from kinect.src.subscribe_color import SingleImageSubscriber

class ARMarkPose(SingleImageSubscriber):

    def __init__(self, topic, ):
        super().__init__(topic)

        self.mtx = np.array([1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0]).reshape(3,3)
        self.dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.marker_length = 0.1
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.roll_pub = rospy.Publisher('robot_roll', Float32MultiArray, queue_size=10)
        self.lateral_pub = rospy.Publisher('robot_lateral', Float32MultiArray, queue_size=10)
        self.longitudinal_pub = rospy.Publisher('robot_longitudinal', Float32MultiArray, queue_size=10)

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

    def rgb_callback(self, rgb):
        start = time.time()
        frame = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        xyzs = []
        Rs = []
        xs, ys, zs = [], [], []

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)
        if len(corners) == 0:
            # roll_array = Float32MultiArray(data=[0])
            # lateral_array = Float32MultiArray(data=[0])
            # longitudinal_array = Float32MultiArray(data=[0])

            # self.roll_pub.publish(roll_array)
            # self.lateral_pub.publish(lateral_array)
            # self.longitudinal_pub.publish(longitudinal_array)
            return
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.mtx, self.dist)
        # マーカごとに姿勢を計算
        for i in range(len(corners)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([corners[i]], self.marker_length, self.mtx, self.dist)
            
            # 回転行列の計算
            try:
                R = cv2.Rodrigues(rvec)[0]  # 回転ベクトル -> 回転行列
                # ここで R を使用して姿勢を利用するか、他の処理を行う
            except cv2.error as e:
                import ipdb;ipdb.set_trace()
                print(f"Error calculating rotation matrix for marker {ids[i][0]}: {str(e)}")
            R_T = R.T
            T = tvec[0].T
            xyz = np.dot(R_T, - T).squeeze()
            _x, _y, _z = xyz
            if ids[i]==1:
                _x -= 0.15
            elif ids[i]==2:
                _x += 0.15
            _y -=0.13
            xs.append(_x)# lateral
            ys.append(_y)# longitudinal
            zs.append(_z)

            xyzs.append(xyz)

            quat = Rotation.from_matrix(R).as_quat()
            _, _, roll = Quaternion(quat).yaw_pitch_roll
            Rs.append(roll)
            # Rs.append(R)
            # rpy = np.deg2rad(cv2.RQDecomp3x3(R_T)[0])
            # # RPY.append(rpy)
            # vx = np.dot(R_T, np.array([1,0,0]))
            # vy = np.dot(R_T, np.array([0,1,0]))
            # vz = np.dot(R_T, np.array([0,0,1]))

            # pose1 = self.plot_pose(xyz, vx, vy, vz, elev=105, azim=270)
            # pose2 = self.plot_pose(xyz, vx, vy, vz, elev=165, azim=270)
            # ---- 描画
            cv2.aruco.drawDetectedMarkers(frame, corners[i:i+1], ids[i:i+1], (0,255,255))
            cv2.aruco.drawAxis(frame, self.mtx, self.dist, rvec, tvec, self.marker_length/2)
            # break
            # print(len(corners))
        cv2.imshow('frame', frame)

        # cv2.imshow('front', pose1)
        # cv2.imshow('front_forward', pose2)
        # print('.............')
        # print(f"tvec : {ys}")
        # print(f'rot : {Rs}')
        roll_array = Float32MultiArray(data=Rs)
        lateral_array = Float32MultiArray(data=xs)
        longitudinal_array = Float32MultiArray(data=ys)

        self.roll_pub.publish(roll_array)
        self.lateral_pub.publish(lateral_array)
        self.longitudinal_pub.publish(longitudinal_array)
        # np.save('/root/catkin_ws/src/kinect/pose/tvec.npy', np.array(xyz))
        # np.save('/root/catkin_ws/src/kinect/pose/rot.npy', np.array(R))
        # print(1/(time.time()-start))
        cv2.waitKey(1)

    




if __name__=="__main__":
    rospy.init_node('est_pos')
    print('Start')
    image_topic = "/kinect2/hd/image_color_rect"
    sub = ARMarkPose(image_topic)
    rospy.spin()
    # sub.est_pose()
    