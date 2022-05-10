# -*- encoding: utf-8 -*-


import argparse
import numpy as np
import rospy
import roslib
import pdb
# import cv
import cv2
import cv_bridge
import rospkg

import numpy
import math
import os
import sys
import string
import time
import random
import tf
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from std_msgs.msg import (
    Header,
    Empty,
)

# load the package manifest
roslib.load_manifest("activerobots")

# initialise ros node
rospy.init_node("pick_and_place", anonymous=True)

# directory used to save analysis images
image_directory = os.getenv("HOME") + "/Ball/"


# locate class
class locate():
    def __init__(self, arm, distance):
        global image_directory
        self.limb = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # image directory
        self.image_dir = image_directory

        # flag to control saving of analysis images
        self.save_images = True

        # required position accuracy in metres
        self.ball_tolerance = 0.005  # .01
        # self.tray_tolerance = 0.05

        # number of balls found
        self.balls_found = 0

        # start positions
        self.ball_store_x = 0.40  # x     = front back
        self.ball_store_y = 0.20  # y     = left right
        self.ball_store_z = 0.15  # z     = up down
        self.ball_pos_x = 0.40  # x     = front back
        self.ball_pos_y = 0.00  # y     = left right
        self.ball_pos_z = 0.15  # z     = up down
        self.roll = -1.0 * math.pi  # roll  = horizontal
        self.pitch = 0.0 * math.pi  # pitch = vertical
        self.yaw = 0.0 * math.pi  # yaw   = rotation

        self.pose = [self.ball_pos_x, self.ball_pos_y, self.ball_pos_z, \
                     self.roll, self.pitch, self.yaw]

        # camera parameters (NB. other parameters in open_camera)
        self.cam_calib = 0.0025  # 0.0025 meters per pixel at 1 meter
        self.cam_x_offset = 0.04  # 0.04 camera gripper offset
        self.cam_y_offset = -0.01
        self.width = 800  # Camera resolution
        self.height = 800
        self.pose_adj_x = 0.0
        self.pose_adj_y = 0.0
        self.target=[350,150]
        # Hough circle accumulator threshold and minimum radius.
        self.hough_accumulator = 35
        self.hough_min_radius = 15
        self.hough_max_radius = 35

        # canny image
        # self.canny = cv.CreateImage((self.width, self.height), 8, 1)
        self.canny = np.zeros((self.height, self.width, 1), np.uint8)

        # Canny transform parameters
        self.canny_low = 45
        self.canny_high = 150

        # minimum ball tray area
        # self.min_area = 35000

        # callback image
        # self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)
        self.cv_image = np.zeros((self.height, self.width, 3), np.uint8)

        # colours
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

        # Enable the actuators
        baxter_interface.RobotEnable().enable()

        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.5)
        self.other_limb_interface.set_joint_position_speed(0.5)

        # create image publisher to head monitor
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)

        # calibrate the gripper
        self.gripper.calibrate()

        # display the start splash screen
        self.splash_screen("Ball", "Pick and Place")

        # subscribe to required camera
        self.subscribe_to_camera(self.limb)

        # distance of arm to table and ball tray
        self.distance = .5
        self.tray_distance = .0625  # 075

        # move other arm out of harms way
        print("moving out of harms way")
        if arm == "left":
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))

    # convert Baxter point to image pixel
    def baxter_to_pixel(self, pt, dist):
        x = (self.width / 2) \
            + int((pt[1] - (self.pose[1] + self.cam_y_offset)) / (self.cam_calib * dist))
        y = (self.height / 2) \
            + int((pt[0] - (self.pose[0] + self.cam_x_offset)) / (self.cam_calib * dist))

        return (x, y)

    # convert image pixel to Baxter point
    def pixel_to_baxter(self, px, dist):
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist) \
            + self.pose[1] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist) \
            + self.pose[0] + self.cam_y_offset

        return (x, y)

    # camera call back function
    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError, e:
            print
            e

        # 3ms wait
        cv2.waitKey(3)

    # left camera call back function
    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    # right camera call back function
    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    # head camera call back function
    def head_camera_callback(self, data):
        self.camera_callback(data, "Head Camera")

    # create subscriber to the required camera
    def subscribe_to_camera(self, camera):
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

    # Convert cv image to a numpy array
    def cv2array(self, im):
        depth2dtype = {cv2.IPL_DEPTH_8U: 'uint8',
                       cv2.IPL_DEPTH_8S: 'int8',
                       cv2.IPL_DEPTH_16U: 'uint16',
                       cv2.IPL_DEPTH_16S: 'int16',
                       cv2.IPL_DEPTH_32S: 'int32',
                       cv2.IPL_DEPTH_32F: 'float32',
                       cv2.IPL_DEPTH_64F: 'float64'}

        arrdtype = im.depth
        a = numpy.fromstring(im.tostring(),
                             dtype=depth2dtype[im.depth],
                             count=im.width * im.height * im.nChannels)
        a.shape = (im.height, im.width, im.nChannels)

        return a

    # find next object of interest
    def find_next_ball_pos(self, ball_data, iteration):
        # if only one object then return object found
        print(len(ball_data))
        if len(ball_data) == 1:
            return ball_data[0]

        # sort objects right to left
        od = []
        for i in range(len(ball_data)):
            od.append(ball_data[i])

        od.sort()

        # if one ball is significantly to the right of the others
        if od[1][0] - od[0][0] > 30:  # if ball significantly to right of the others
            return od[0]  # return right most ball
        elif od[1][1] < od[0][1]:  # if right most ball below second ball
            return od[0]  # return lower ball
        else:  # if second ball below right most ball
            return od[1]  # return lower ball

    # Use Hough circles to find ball centres (Only works with round objects)
    def hough_it(self, n_ball, iteration):
        # create gray scale image of the ball
        # gray_image = cv2.cv.CreateImage((cv.GetSize(cv.fromarray(self.cv_image))), 8, 1)#(self.width, self.height)

        gray_image = np.zeros((self.height, self.width, 1), np.uint8)
        # cv2.cvtColor(cv2.putText(self.cv_image), gray_image, cv2.CV_BGR2GRAY)
        gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        # create gray scale array of the ball
        # gray_array = self.cv2array(gray_image)
        gray_array = np.array(gray_image)
        # find Hough circles
        circles = cv2.HoughCircles(gray_array, cv2.HOUGH_GRADIENT, 1, 40, param1=50, \
                                   param2=self.hough_accumulator, minRadius=self.hough_min_radius, \
                                   maxRadius=self.hough_max_radius)

        # Check for at least one ball found
        if circles is None:
            # display no ball found message on head display
            self.splash_screen("no ball", "found")
            # no point in continuing so exit with error message
            print
            "ERROR - hough_it - No balls found"
            raw_input("Press Enter to finish: ")
            sys.exit("Finished")

        circles = numpy.uint16(numpy.around(circles))

        ball_data = {}
        n_balls = 0

        circle_array = numpy.asarray(self.cv_image)

        # check if golf ball is in storage area
        for i in circles[0, :]:
            # convert to baxter coordinates
            ball = self.pixel_to_baxter((i[0], i[1]), self.tray_distance)

            if self.is_near_storage(ball):
                # draw the outer circle in red
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
                # draw the center of the circle in red
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 0, 255), 3)
            elif i[1] > 800 or i[0] > 600:
                # draw the outer circle in red
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
                # draw the center of the circle in red
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 0, 255), 3)
            else:
                # draw the outer circle in green
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle in green
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 255, 0), 3)

                ball_data[n_balls] = (i[0], i[1], i[2])
                n_balls += 1

        # circle_image = cv2.putText(circle_array)
        # cv2.ShowImage("Hough Circle", circle_image)

        cv2.imshow("Hough Circle", circle_array)

        # 3ms wait
        cv2.waitKey(3)

        # display image on head monitor
        # font     = cv2.InitFont(cv2.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        position = (30, 60)
        s = "Searching for balls"
        #############cv2.PutText(circle_image, s, position, font, self.white)
        cv2.putText(circle_array, s, position, font,
                    fontScale=1,
                    color=self.white,
                    thickness=2)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(circle_array, encoding="bgr8")
        self.pub.publish(msg)

        if self.save_images:
            # save image of Hough circles on raw image
            file_name = self.image_dir \
                        + "hough_circle_" + str(n_ball) + "_" + str(iteration) + ".jpg"
            cv2.imwrite(file_name, circle_array)

        # Check for the existence of the ball
        if n_balls == 0:  # no ball found
            # display no ball found message on head display
            self.splash_screen("no ball found")
            # less than 12 balls found, no point in continuing, exit with error message
            print
            "ERROR - hough_it - No ball found"
            raw_input("Press Enter to finish: ")
            sys.exit("Finished")

        # select next ball and find it's position
        next_ball = self.find_next_ball_pos(ball_data, iteration)

        return next_ball

    # move a limb
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 6.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print
            "invalid requested move =", rpy_pose
            # sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")
            self.pose = [self.ball_pos_x, self.ball_pos_y, self.ball_pos_z, \
                         self.roll, self.pitch, self.yaw]
            self.baxter_ik_move(limb, self.pose)

        if self.limb == limb:  # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved  self.pose[2]
            self.pose = [position[0], position[1], position[2], self.pose[3], self.pose[4], self.pose[5]]

    # find distance of limb from nearest line of sight object
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)

    # update pose in x and y direction
    def update_pose(self, dx, dy):
        x = (self.pose[0] + dx)
        y = (self.pose[1] + dy)
        # pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
        pose = [x, self.pose[1], self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)
        pose = [self.pose[0], y, self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)

    # print all 6 arm coordinates (only required for programme development)
    def print_arm_pose(self):

        pi = math.pi

        quaternion_pose = self.limb_interface.endpoint_pose()
        position = quaternion_pose['position']
        quaternion = quaternion_pose['orientation']
        euler = tf.transformations.euler_from_quaternion(quaternion)

        print
        print
        "             %s" % self.limb
        print
        'front back = %5.4f ' % position[0]
        print
        'left right = %5.4f ' % position[1]
        print
        'up down    = %5.4f ' % position[2]
        r = euler[0] * (180 / pi)
        p = euler[1] * (180 / pi)
        y = euler[2] * (180 / pi)
        print
        'roll       = %5.4f radians %5.4f degrees' % (euler[0], r)
        print
        'pitch      = %5.4f radians %5.4f degrees' % (euler[1], p)
        print
        'yaw        = %5.4f radians %5.4f degrees' % (euler[2], y)
        return

    # find all the ball and place it in the ball tray
    def pick_and_place(self):
        n_ball = 0
        while n_ball < 1:
            n_ball += 1
            iteration = 0

            # use Hough circles to find the ball and select one ball
            ball = self.hough_it(n_ball, iteration)
            print('ball', ball)

            # find displacement of ball from centre of image
            while np.linalg.norm(ball[0, 1] - self.target) > 10:
                pixel_dx = self.target[0] - ball[0]
                pixel_dy = self.target[1] - ball[1]

                x_offset = + pixel_dy * self.cam_calib * self.tray_distance
                y_offset = - pixel_dx * self.cam_calib * self.tray_distance

                # update pose and find new ball data
                self.update_pose(x_offset, y_offset)

                font = cv2.FONT_HERSHEY_SIMPLEX
                position = (30, 60)
                s = "Picking up the ball"
                cv2.putText(self.cv_image, s, position, font,
                            fontScale=1,
                            color=self.white,
                            thickness=2)
                msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                self.pub.publish(msg)
                self.limb_interface.set_joint_position_speed(0.1)
                self.baxter_ik_move(self.limb, self.pose)

            pose = (self.pose[0],
                    self.pose[1],
                    self.pose[2] - 0.27,
                    self.pose[3],
                    self.pose[4]
                    )
            self.baxter_ik_move(self.limb, pose)

            # close the gripper
            self.gripper.close()
            time.sleep(.400)

            s = "Moving to ball tray"
            # cv.PutText(cv.fromarray(self.cv_image), s, position, font, self.white)
            cv2.putText(self.cv_image, s, position, font,
                        fontScale=1,
                        color=self.white,
                        thickness=2)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
            self.pub.publish(msg)

            pose = (self.pose[0],
                    self.pose[1],
                    self.pose[2] + 0.29,
                    self.pose[3],
                    self.pose[4],
                    self.yaw)
            self.baxter_ik_move(self.limb, pose)

            pose = (self.ball_store_x,
                    self.ball_store_y,
                    self.ball_store_z,
                    self.pose[3],
                    self.pose[4],
                    self.yaw)
            self.baxter_ik_move(self.limb, pose)

            cv2.putText(self.cv_image, s, position, font,
                        fontScale=1,
                        color=self.white,
                        thickness=2)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
            self.pub.publish(msg)

            # move down
            pose = (self.ball_store_x,
                    self.ball_store_y,
                    self.pose[2] - 0.20,
                    self.pose[3],
                    self.pose[4],
                    self.pose[5])
            self.baxter_ik_move(self.limb, pose)

            # display current image on head display
            s = "Placing the ball in ball tray"
            cv2.putText(self.cv_image, s, position, font,
                        fontScale=1,
                        color=self.white,
                        thickness=2)

            msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
            self.pub.publish(msg)

            # open the gripper
            self.gripper.open()
            time.sleep(.400)

            # prepare to look for next ball
            pose = (self.ball_pos_x,
                    self.ball_pos_y,
                    self.ball_pos_z,
                    -1.0 * math.pi,
                    0.0 * math.pi,
                    0.0 * math.pi)
            self.baxter_ik_move(self.limb, pose)

        # display found on head display
        self.splash_screen("Finished")

        print
        "Finished"

    # display message on head display
    def splash_screen(self, s1, s2):
        splash_array = numpy.zeros((self.height, self.width, 3), numpy.uint8)
        # font = cv2.InitFont(cv2.CV_FONT_HERSHEY_SIMPLEX, 3.0, 3.0, 9)
        font = cv2.FONT_HERSHEY_SIMPLEX

        ((text_x, text_y), baseline) = cv2.getTextSize(s1, font, 3, 3)
        org = ((self.width - text_x) / 2, (self.height / 3) + (text_y / 2))
        cv2.putText(splash_array, s1, org, cv2.FONT_HERSHEY_SIMPLEX, 3.0, \
                    self.white, thickness=7)

        ((text_x, text_y), baseline) = cv2.getTextSize(s2, font, 3, 3)
        org = ((self.width - text_x) / 2, ((2 * self.height) / 3) + (text_y / 2))
        cv2.putText(splash_array, s2, org, cv2.FONT_HERSHEY_SIMPLEX, 3.0, \
                    self.white, thickness=7)

        # 3ms wait
        cv2.waitKey(3)

        msg = cv_bridge.CvBridge().cv2_to_imgmsg(splash_array, encoding="bgr8")
        self.pub.publish(msg)


# read the setup parameters from setup.dat
def get_setup():
    global image_directory
    file_name = image_directory + "setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: golf_setup must be run before golf")

    # find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    # find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance


def main():
    # get setup parameters
    limb, distance = get_setup()
    print("ballpick")
    # create locate class instance
    locator = locate(limb, distance)

    locator.gripper.open()

    # move close to the ball tray
    locator.pose = (locator.ball_store_x,
                    locator.ball_store_y,
                    locator.ball_store_z,
                    locator.roll,
                    locator.pitch,
                    locator.yaw)
    locator.baxter_ik_move(locator.limb, locator.pose)

    # find the golf ball and place it in the storage area
    locator.pose = (locator.ball_pos_x,
                    locator.ball_pos_y,
                    locator.ball_pos_z,
                    locator.roll,
                    locator.pitch,
                    locator.yaw)
    locator.baxter_ik_move(locator.limb, locator.pose)
    locator.pick_and_place()


if __name__ == "__main__":
    main()
