from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

def detectBall(visualize = True):
    # define the lower and upper boundaries of the "green" ball in the HSV color space
    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)
    # grab the reference to the webcam
    vs = VideoStream(src=0).start()
    # if we want to threshold out smaller contours
    rad_thresh = 0

    # allow the camera or video file to warm up
    time.sleep(2.0)

    # keep looping
    while True:
        # grab the current frame
        frame = vs.read()

        # resize the frame, blur it, and convert it to the HSVcolor space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green"
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        # perform dilations and erosions to remove any small blobs left in the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask
            c = max(cnts, key=cv2.contourArea)
            # compute the minimum enclosing circle and centroid
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > rad_thresh: # tuning param
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        if visualize:
            # show the frame to our screen
            cv2.imshow("Frame", frame)
        
        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

    vs.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detectBall()
