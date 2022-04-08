#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PointStamped
import matplotlib.pyplot as plt
import numpy as np

class FakeBallsNode():
  def __init__(self):
    # Initial positions
    self._xi = 0
    self._yi = 0
    self._zi = 0
    # Initial velocities
    self._vx = 1
    self._vy = 1
    self._vz = 10
    # Gravity
    self._g = -9.81
    # Final time
    self._tf = 1
    # Time delta
    self._dt = 0.1
    # Standard deviations for Gaussian noise
    self._sigma_x = 0.1
    self._sigma_y = 0.1
    self._sigma_z = 0.1

    self._rate = rospy.Rate(1/self._dt)
    self._points_pub = rospy.Publisher('/ball_detections', PointStamped, queue_size=1)
    self._seq = 0
 
    print('Generating points with the following parameters:')
    print('(x, y, z) = ({}, {}, {})'.format(self._xi, self._yi, self._zi))
    print('(vx, vy, vz) = ({}, {}, {})'.format(self._vx, self._vy, self._vz))
    print('(dt, tf) = ({}, {})'.format(self._dt, self._tf))
    print('(sx, sy, sz) = ({}, {}, {})'.format(self._sigma_x, self._sigma_y, self._sigma_z))

    self._trajectory = []
    t = 0
    # Plotting arrays
    ts = []
    xs = []
    xs_nominal = []
    ys = []
    ys_nominal = []
    zs = []
    zs_nominal = []
    while t <= self._tf:
      # Compute nominal values
      x = self._vx*t + self._xi
      xs_nominal.append(x)
      y = self._vy*t + self._yi
      ys_nominal.append(y)
      z = self._g*t*t + self._vz*t + self._zi
      zs_nominal.append(z)
      # Add Gaussian noise
      noise = np.random.normal(scale=[self._sigma_x, self._sigma_y, self._sigma_z])
      x += noise[0]
      y += noise[1]
      z += noise[2]
      ts.append(t)
      xs.append(x)
      ys.append(y)
      zs.append(z)
      self._trajectory.append(Point())
      self._trajectory[-1].x = x
      self._trajectory[-1].y = y
      self._trajectory[-1].z = z
      t += self._dt

    # plt.figure()
    # plt.plot(ts,xs)
    # plt.plot(ts,xs_nominal)
    # plt.figure()
    # plt.plot(ts,ys)
    # plt.plot(ts,ys_nominal)
    # plt.figure()
    # plt.plot(ts,zs)
    # plt.plot(ts,zs_nominal)
    # plt.show()

  def main_loop(self):
    while not rospy.is_shutdown():
      msg = PointStamped()
      msg.header.frame_id = 'base_link'
      msg.header.seq = self._seq
      msg.header.stamp = rospy.Time.now()
      if self._seq < len(self._trajectory):
        msg.point = self._trajectory[self._seq]
        self._points_pub.publish(msg)
        self._seq += 1
      self._rate.sleep()


if __name__ == '__main__':
  rospy.init_node('fake_balls_node')
  fake_balls_node = FakeBallsNode()
  fake_balls_node.main_loop()
