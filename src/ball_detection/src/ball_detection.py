#!/usr/bin/env python

from concurrent.futures import thread
import rospy
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2
import imutils
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pathlib
import os
import matplotlib.pyplot as plt

class BallDetector:

    def __init__(self) -> None:
        self.ball_pos_msg = PointStamped()
        filepath = pathlib.Path(__file__).parent.resolve()
        self.intrinsics = CameraIntrinsics.load(os.path.join(filepath, '../config/azure_kinect.intr'))
        self.extrinsics = RigidTransform.load(os.path.join(filepath, '../config/azure_kinect_overhead/azure_kinect_overhead_to_world.tf'))
        self.rgb_map = None
        self.depth_map = None

        self.rgb_header = None
        self.depth_header = None

        self.marked_image = None

        self.bridge = CvBridge()

        self.rgb_sub = rospy.Subscriber(
        "/rgb/image_raw",
        Image, 
        self.rgb_clbk)

        self.depth_sub = rospy.Subscriber(
        "/depth_to_rgb/image_raw",
        Image, 
        self.depth_clbk)

        self.pub = rospy.Publisher(
        "/ball_detections", 
        PointStamped,
        queue_size=2)

    def rgb_clbk(self, msg):
        self.rgb_map = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        self.rgb_map = self.rgb_map[:,:,:3]
        self.rgb_header = msg.header
    
    def depth_clbk(self, msg):
        self.depth_map = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width, 1)
        self.depth_header = msg.header

    def detectBall(self):
        
        if self.rgb_map is None or self.depth_map is None:
            print('No images')
            return
        # define the lower and upper boundaries of the "green" ball in the HSV color space
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)

        grey_Upper = (40)

        delta_t = self.rgb_header.stamp.to_sec() - self.depth_header.stamp.to_sec()
        #print(delta_t)

        # if we want to threshold out smaller contours
        rad_thresh = 0

        # resize the frame, blur it, and convert it to the HSVcolor space
        blurred = cv2.GaussianBlur(self.rgb_map, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green"
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        # perform dilations and erosions to remove any small blobs left in the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask
            c = max(cnts, key=cv2.contourArea)
            # compute the minimum enclosing circle and centroid
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            self.marked_image = cv2.circle(blurred, (int(x),int(y)), int(radius), (255, 255, 255), 2)
            cv2.imshow('Track', self.marked_image)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                return -1
            # only proceed if the radius meets a minimum size
            # if radius > rad_thresh: # tuning param
            ball_center = self.get_point_in_world(int(x), int(y), self.depth_map, radius)

            # Set up message
            self.ball_pos_msg.header = self.rgb_header
            self.ball_pos_msg.point.x = ball_center[0]
            self.ball_pos_msg.point.y = ball_center[1]
            self.ball_pos_msg.point.z = ball_center[2]
            self.pub.publish(self.ball_pos_msg)

            return 0

    def get_point_in_world(self, center_x, center_y, depth_image, radius): 
        # thresh = max(25, int(2.5 * radius))
        thresh = 50
        object_center = Point(np.array([center_x, center_y]), 'azure_kinect_overhead')
        object_depth = depth_image[max(center_y-thresh, 0): center_y+thresh, max(0, center_x-thresh): center_x+thresh, 0] / 1000

        if object_depth[object_depth != 0].size != 0:
            object_depth2 = np.min(np.where(object_depth==0, 1e6, object_depth)) / 1000
            print(object_depth2)
        else:
            object_depth2 = depth_image[center_y, center_x, 0] / 1000
        object_center_point_in_world = self.extrinsics * self.intrinsics.deproject_pixel(object_depth2, object_center)    

        return object_center_point_in_world

    def main_loop(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            cont = self.detectBall()
            rate.sleep()
            if cont == -1:
                break


if __name__ == '__main__':
    rospy.init_node('ball_detector')
    cv2.namedWindow('Track')
    ball_detector = BallDetector()
    ball_detector.main_loop()