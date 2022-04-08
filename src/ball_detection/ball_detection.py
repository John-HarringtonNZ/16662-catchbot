import rospy
from geometry_msgs.msg import PointStamped
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

class BallDetector:

    def __init__(self) -> None:

        self.ball_pos_msg = PointStamped()

        self.P = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0]])

        self.sub = rospy.Subscriber(
        "/kinect_frame", # TODO: check topic 
        PointStamped, 
        self.detectBall)

        self.pub = rospy.Publisher(
        "/ball_detections", 
        PointStamped,
        queue_size=2)


    def detectBall(self, msg):
        # define the lower and upper boundaries of the "green" ball in the HSV color space
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)
        
        # current rgb frame
        rgbframe = msg.rgbmap # TODO: check msg 
        dframe = msg.dmap # TODO: check msg 

        # if we want to threshold out smaller contours
        rad_thresh = 0

        # resize the frame, blur it, and convert it to the HSVcolor space
        rgbframe = imutils.resize(rgbframe, width=600)
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
        if len(cnts) > 0:
            # find the largest contour in the mask
            c = max(cnts, key=cv2.contourArea)
            # compute the minimum enclosing circle and centroid
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            z = dframe[int(x)][int(y)]

            # only proceed if the radius meets a minimum size
            # if radius > rad_thresh: # tuning param

            #TODO: get extrinsic calibration
            X = np.array([x, y, z, 1]).T
            ball_X = self.P@X

            ball_x = ball_X[0]
            ball_y = ball_X[1]
            ball_z = ball_X[2]

            # Set up message
            self.ball_pos_msg.header = msg.header
            self.ball_pos_msg.point.x = ball_x
            self.ball_pos_msg.point.y = ball_y
            self.ball_pos_msg.point.z = ball_z

            self.pub.publish(self.intersect_pos_msg)
            

if __name__ == '__main__':
    rospy.init_node('ball_detector')
    BallDetector()
    rospy.spin()
