import rospy
from geometry_msgs.msg import PointStamped
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class BallDetector:

    def __init__(self) -> None:

        self.ball_pos_msg = PointStamped()
        self.intrinsics = azure_kinect_intrinsics = CameraIntrinsics.load('config/azure_kinect.intr')
        self.extrinsics = azure_kinect_to_world_transform = RigidTransform.load('config/azure_kinect_overhead/azure_kinect_overhead_to_world.tf')
        self.rgb_map = None
        self.depth_map = None

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(
        "/rgb_to_depth",
        PointStamped, 
        self.depth_clbk)

        self.sub = rospy.Subscriber(
        "/depth_to_rgb",
        PointStamped, 
        self.rgb_clbk)

        self.pub = rospy.Publisher(
        "/ball_detections", 
        PointStamped,
        queue_size=2)

    def rgb_clbk(self, msg):
        self.rgb_map  = self.bridge.imgmsg_to_cv2(msg.raw_image.data, msg.encoding)
    
    def depth_clbk(self, msg):
        self.depth_map  = self.bridge.imgmsg_to_cv2(msg.raw_image.data, msg.encoding)

    def detectBall(self, msg):
        # define the lower and upper boundaries of the "green" ball in the HSV color space
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)

        # if we want to threshold out smaller contours
        rad_thresh = 0

        # resize the frame, blur it, and convert it to the HSVcolor space
        rgbframe = imutils.resize(self.rgb_map, width=600)
        blurred = cv2.GaussianBlur(rgbframe, (11, 11), 0)
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
        if len(cnts) > 0 and self.rgb_map is not None and self.depth_map is not None:
            # find the largest contour in the mask
            c = max(cnts, key=cv2.contourArea)
            # compute the minimum enclosing circle and centroid
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # only proceed if the radius meets a minimum size
            # if radius > rad_thresh: # tuning param

            #TODO: get extrinsic calibration
            ball_center = self.get_object_center_point_in_world(x, y, self.depth_map)


            # X = np.array([x, y, 1]).T
            # ball_X = self.P@X

            # ball_x = ball_X[0]/ball_X[2]
            # ball_y = ball_X[1]/ball_X[2]
            # ball_z = dframe[int(x)][int(y)]

            # Set up message
            self.ball_pos_msg.header = msg.header
            self.ball_pos_msg.point.x = ball_center[0]
            self.ball_pos_msg.point.y = ball_center[1]
            self.ball_pos_msg.point.z = ball_center[2]

            self.pub.publish(self.intersect_pos_msg)

    def get_point_in_world(self, object_image_center_x, object_image_center_y, depth_image): 
        
        object_center = Point(np.array([object_image_center_x, object_image_center_y]), 'azure_kinect_overhead')
        object_depth = depth_image[object_image_center_y, object_image_center_x]
        # print("x, y, z: ({:.4f}, {:.4f}, {:.4f})".format(
        #     object_image_center_x, object_image_center_y, object_depth))
        object_center_point_in_world = self.extrinsics * self.intrinsics.deproject_pixel(object_depth, object_center)    
        # print(object_center_point_in_world)

        return object_center_point_in_world
                

if __name__ == '__main__':
    rospy.init_node('ball_detector')
    BallDetector()
    rospy.spin()
