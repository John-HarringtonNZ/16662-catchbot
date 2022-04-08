import rospy

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

if __name__ == '__main__':
  pass
