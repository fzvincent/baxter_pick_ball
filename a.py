
import argparse
import rospy
import roslib
import pdb
import rospkg
import numpy as np
import cv_bridge
import cv2
import numpy
import math
import os
import sys
import string
import time
import random
import tf
import baxter_interface

class ballpick():
    def __init__(self, arm, distance):
        self.limb = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # calibrate the gripper
        self.gripper.calibrate()
        # subscribe to required camera
        self.subscribe_to_camera(self.limb)


        self.cv_image = np.zeros((self.height, self.width, 3), np.uint8)

        # Hough circle accumulator threshold and minimum radius.
        self.hough_accumulator = 30
        self.hough_minr = 10
        self.hough_maxr = 30

    def baxter_move(self, limb, rpy_pose):

    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
        except (cv_bridge.CvBridgeError )as e:
            print e

    # left camera call back function
    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    # right camera call back function
    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    # head camera call back function
    def head_camera_callback(self, data):
        self.camera_callback(data, "Head Camera")
    def hough_it(self):
        # gray_image = np.zeros((self.height, self.width, 1), np.uint8)
        # cv.CvtColor(cv.fromarray(self.cv_image), gray_image, cv.CV_BGR2GRAY)
        gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray_image, cv2.CV_HOUGH_GRADIENT, 1, 40, param1=50, \
                                   param2=self.hough_accumulator, minRadius=self.hough_minr, \
                                   maxRadius=self.hough_maxr)
    def pick_and_place(self):
        ball_position = self.hough_it()
        
def main():
    


    
    ballpick.gripper.open()

    ballpick.pose = (ballpick.golf_ball_x,
                    ballpick.golf_ball_y,
                    ballpick.golf_ball_z,
                    ballpick.roll,
                    ballpick.pitch,
                    ballpick.yaw)
    ballpick.baxter_position(ballpick.limb, ballpick.pose)
    ballpick.pick_and_place()

if __name__ == "__main__":
    main()