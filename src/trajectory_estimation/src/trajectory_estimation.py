#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
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
      print('Initializing trajectory estimation')
      self.detections = np.array([])
      self.times = np.array([])
      self.reference_time = 0
      self.reference_z = 0.25 # meters off base link plane

      self.max_x = 2.0
      self.min_x = 0.7

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
      self.traj_pub  = rospy.Publisher(
        "/trajectory_marker",
        Marker,
        queue_size=1
      )

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
    # self.intersect_pos_msg.header.stamp = rospy.Time.now()
    # self.intersect_pos_msg.point.x = 0.45
    # self.intersect_pos_msg.point.y = -0.25
    # self.intersect_pos_msg.point.z = 0.4
    # self.pub.publish(self.intersect_pos_msg)
    # return

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
    z_poly = np.polynomial.polynomial.Polynomial(z_fit)

    # Publish trajectory points for visualization
    traj_points = []
    for t in np.linspace(self.times[0], intersection_time, 15):
      traj_points.append(Point())
      traj_points[-1].x = x_poly(t)
      traj_points[-1].y = y_poly(t)
      traj_points[-1].z = z_poly(t)

    traj_marker = Marker()
    traj_marker.type = traj_marker.POINTS
    traj_marker.header = msg.header
    traj_marker.points = traj_points
    traj_marker.action = traj_marker.ADD
    traj_marker.scale.x = 0.05
    traj_marker.scale.y = 0.05
    traj_marker.scale.z = 0.05
    traj_marker.color.r = 1.0
    traj_marker.color.a = 1.0
    traj_marker.pose.orientation.w = 1.0
    self.traj_pub.publish(traj_marker)

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