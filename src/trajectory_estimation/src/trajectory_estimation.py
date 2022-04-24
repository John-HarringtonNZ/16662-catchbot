#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import numpy as np

'''
geometry_msgs/Point subscriber
Store points in a numpy matrix
3 calls to np.polyfit, 
  * x linear with time
  * y linear with time
  * z parabolic with time
Find parabola intersection with reference plane (intersection with certain z-height) -> get t
Plug in t to get x, y location
Publish x,y,z location as a geometry_msgs/Point
'''

class TrajectoryEstimator:

  def __init__(self) -> None:
      self.detections = np.array([])
      self.times = np.array([])
      self.reference_time = 0
      self.reference_z = 0.0 # meters off base link plane

      self.max_x = 2.0
      self.min_x = 0.6

      self.last_xy = None
      self.pub_threshold = 0.02 # Only update publisher if new intersect is different enough

      self.intersect_pos_msg = PointStamped()

      self.sub = rospy.Subscriber(
        "/ball_detections", 
        PointStamped, 
        self.update_traj_estimate)
      self.pub = rospy.Publisher(
        "/intersect_position", 
        PointStamped,
        queue_size=2)

  def reset(self, header):
    print('Resetting')
    self.detections = np.array([])
    self.times = np.array([])
    self.last_xy = None
    self.reference_time = 0
    # home_pos_msg = PointStamped()
    # home_pos_msg.header = header
    # home_pos_msg.point.x = 0
    # home_pos_msg.point.y = 0
    # home_pos_msg.point.z = 0
    # self.pub.publish(home_pos_msg)

  def update_traj_estimate(self, msg):
    print(msg.point.x)
    if msg.point.x > self.max_x or msg.point.x < self.min_x:
      self.reset(msg.header)
      return

    # If not initialized, start
    if self.detections.size == 0:

      self.detections = np.array([[msg.point.x, msg.point.y, msg.point.z - self.reference_z]])
      self.reference_time = msg.header.stamp.to_sec()

    else: 
      
      self.detections = np.append(
        self.detections,
          [[
            msg.point.x,
            msg.point.y,
            msg.point.z - self.reference_z]], axis=0)
    
    self.times = np.append(
      self.times,
      np.array(msg.header.stamp.to_sec() - self.reference_time))

    # Don't compute if not enough points
    if self.detections.shape[0] < 4:
      return

    # Fit x and y linearly, and z parabolicly
    x_fit = np.polynomial.polynomial.polyfit(
      self.times, self.detections[:,0], 1)
    y_fit = np.polynomial.polynomial.polyfit(
      self.times, self.detections[:,1], 1)
    z_fit = np.polynomial.polynomial.polyfit(
      self.times, self.detections[:,2], 2)

    # Get intersection point for reference
    z_roots = np.polynomial.polynomial.polyroots(z_fit)
    intersection_time = max(z_roots)

    # Use roots to get estimated x and y location
    x_poly = np.polynomial.polynomial.Polynomial(x_fit)
    y_poly = np.polynomial.polynomial.Polynomial(y_fit)
    x_intersect = x_poly(intersection_time)
    y_intersect = y_poly(intersection_time)

    # Only update message if beyond threshold
    if self.last_xy:
      new_xy = np.array([x_intersect, y_intersect])
      update_dist = np.linalg.norm(new_xy - self.last_xy)
      if update_dist > self.pub_threshold:
        self.last_xy = new_xy
      else:
        return

    # Set up message
    self.intersect_pos_msg.header = msg.header
    self.intersect_pos_msg.point.x = x_intersect
    self.intersect_pos_msg.point.y = y_intersect
    self.intersect_pos_msg.point.z = self.reference_z

    self.pub.publish(self.intersect_pos_msg)


if __name__ == '__main__':
  rospy.init_node('trajectory_estimator')
  TrajectoryEstimator()
  rospy.spin()