#!/usr/bin/env python

import time
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage

from franka_interface_msgs.msg import SensorDataGroup


class PlanningNode:

    TOOL_DELTA_POSE = RigidTransform(
            translation=np.array([0.106, -0.106, -0.01]),
            rotation=np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0,  0, 1]]),
            from_frame="franka_tool",
            to_frame="franka_tool_base"
        )

    # A good pose from where robot can start moving from
    START_POSE = RigidTransform(
                translation=np.array([0.4, 0, 0.4]),
                rotation=np.array([[7.07185286e-01, -7.07028188e-01, -3.36139057e-04],
                                    [-7.07028245e-01, -7.07185062e-01, -5.90835436e-04],
                                    [1.80024788e-04,  6.55489934e-04, -9.99999769e-01]]),
                from_frame="franka_tool",
                to_frame="world"
            )

    def __init__(self, fa) -> None:
        # initialize arm
        self.fa = FrankaArm(with_gripper=False)
        fa.set_tool_delta_pose(self.TOOL_DELTA_POSE)
        self.is_resetting = False
        print('Initialized Arm')

        self.reset()
        self.setup_dynamic()

    def reset(self):
        self.is_resetting = True
        print('Resetting Arm...')
        self.fa.goto_pose(self.START_POSE, duration=5)
        self.fa.wait_for_skill()
        self.is_resetting = False
        print("Reset Complete...")

    def setup_dynamic(self):
        self.BOX_CORNER_MIN = np.array([0.3, -0.3, 0.2])
        self.BOX_CORNER_MAX = np.array([0.5, 0.3, 0.55])

        self.sub = rospy.Subscriber(
            "/intersect_position", 
            PointStamped,
            self.get_goal_position)
        self.pub = rospy.Publisher(
            FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000
        )
        print(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC)

        self.dt = 0.01
        self.rate = rospy.Rate(1 / self.dt)
        self.disp = 0.15
        self.init_time = rospy.Time.now().to_time()
        self.id = 0

        self.timer = rospy.Timer(rospy.Duration(self.dt), self.moveToCatch)
        self.timeout = 5

        curr_pose = self.fa.get_pose()
        self.fa.goto_pose(curr_pose, duration=self.dt, dynamic=True, buffer_time=5, block=False)
        print("Started dynamic mode")


    def get_goal_position(self, goal_pose_msg):
        self.goal_pose_msg = goal_pose_msg
        
    def is_valid_pose(self, pose):
        return np.all(pose.translation <= self.BOX_CORNER_MAX) and np.all(pose.translation >= self.BOX_CORNER_MIN)

    def moveToCatch(self, event):
        curr_pose = self.fa.get_pose()
        if self.goal_pose_msg is None:
            goal_pose = curr_pose
        else:
            goal_pose = RigidTransform(
            translation=np.array([
                self.goal_pose_msg.point.x,
                self.goal_pose_msg.point.y,
                self.goal_pose_msg.point.z
            ]),
            from_frame=curr_pose.from_frame,
            to_frame=curr_pose.to_frame,
        )

        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        # self.fa.goto_pose(curr_pose, duration=self.dt, dynamic=True, buffer_time=5, block=False)
        interpol_pose = curr_pose
        print("Current error: ", np.linalg.norm(curr_pose.translation - goal_pose.translation))
        delta = goal_pose.translation - curr_pose.translation
        delta_norm = np.linalg.norm(delta)
        delta = min(delta_norm, self.disp) * (delta/delta_norm) 
        interpol_pose.translation = curr_pose.translation + delta
        # interpol_pose = goal_pose
        timestamp = rospy.Time.now().to_time() - self.init_time
            
        if not self.is_valid_pose(interpol_pose):
            print("Invalid Pose:", interpol_pose.translation)
            interpol_pose = curr_pose

        traj_gen_proto_msg = PosePositionSensorMessage(
            id=self.id,
            timestamp=timestamp,
            position=interpol_pose.translation,
            quaternion=interpol_pose.quaternion,
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION
            )
        )
        self.pub.publish(ros_msg)


if __name__ == "__main__":

    # rospy.init_node('planning_node', disable_signals=True, log_level=rospy.INFO)
    PlanningNode()
    rospy.spin()
    # print(fa.get_tool_base_pose())
    # Go to start position
    # fa.set_tool_delta_pose(TOOL_DELTA_POSE)
    

    # Create a displacement w.r.to the tool and create a goal pose
    # curr_pose = fa.get_pose()
    # T_delta = RigidTransform(
    #     translation=np.array([0.0, -0.25, 0]),
    #     from_frame=curr_pose.to_frame,
    #     to_frame=curr_pose.to_frame,
    # )
    # goal_pose = T_delta * curr_pose

    # moveToCatch(fa, goal_pose, disp=0.15, dt=0.01)
