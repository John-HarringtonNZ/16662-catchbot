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
            translation=np.array([0.56, 0, 0.4]),
            rotation=np.array([[7.07185286e-01, -7.07028188e-01, -3.36139057e-04],
                                [-7.07028245e-01, -7.07185062e-01, -5.90835436e-04],
                                [1.80024788e-04,  6.55489934e-04, -9.99999769e-01]]),
            from_frame="franka_tool",
            to_frame="world"
        )

    BOX_CORNER_MIN = np.array([0.46, -0.3, 0.2])
    BOX_CORNER_MAX = np.array([0.66, 0.3, 0.7])

    def __init__(self) -> None:
        # initialize arm
        self.fa = FrankaArm(with_gripper=False)
        self.fa.set_tool_delta_pose(self.TOOL_DELTA_POSE)
        self.is_resetting = True

        self.reset()
        print('Initialized Arm')


    def reset(self):
        self.is_resetting = True
        print('Resetting Arm...')
        self.fa.reset_joints()
        self.fa.goto_pose(self.START_POSE, duration=2)
        self.fa.stop_skill()
        self.fa.wait_for_skill()
        self.setup_dynamic()
        self.is_resetting = False
        print("Reset Complete...")

    def setup_dynamic(self):

        self.goal_pose_msg = None
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
        self.disp = 0.2
        self.init_time = rospy.Time.now().to_time()
        self.id = 0
        self.last_valid_pose_time = time.time()

        self.timer = rospy.Timer(rospy.Duration(self.dt), self.moveToCatch)
        self.timeout = 5

        curr_pose = self.fa.get_pose()
        print("Initial pose: ", curr_pose.translation) 
        self.fa.goto_pose(curr_pose, duration=self.dt, dynamic=True, buffer_time=500, block=False)
        print("Started dynamic mode")


    def get_goal_position(self, goal_pose_msg):
        self.goal_pose_msg = goal_pose_msg
        
    def is_valid_pose(self, pose):
        return np.all(pose.translation <= self.BOX_CORNER_MAX) and np.all(pose.translation >= self.BOX_CORNER_MIN)

    def terminate_dynamic(self):
        # exit()
        term_proto_msg = ShouldTerminateSensorMessage(
        timestamp=rospy.Time.now().to_time() - self.init_time, should_terminate=True
        )
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE
            )
        )
        self.pub.publish(ros_msg)
        self.fa.wait_for_skill()

    def moveToCatch(self, event):
        if self.is_resetting:
            return
        curr_pose = self.fa.get_pose()
        if np.any(np.isnan(curr_pose.translation)) or np.any(np.isnan(curr_pose.quaternion)):
            print("Current pose has Nan")
        # print("Current pose: ", curr_pose.translation)        
        if self.goal_pose_msg is None:
            goal_pose = curr_pose
        else:
            goal_pose = RigidTransform(
            translation=np.array([
                # self.START_POSE.translation[0],     # servoing
                self.goal_pose_msg.point.x,
                self.goal_pose_msg.point.y,
                self.goal_pose_msg.point.z
            ]),
            from_frame=curr_pose.from_frame,
            to_frame=curr_pose.to_frame,
        )

        interpol_pose = curr_pose
        # print(goal_pose.translation)
        # print(curr_pose.translation)
        delta = goal_pose.translation - curr_pose.translation
        delta_norm = np.linalg.norm(delta)
        if np.abs(delta_norm) > 1e-3:
            delta = min(delta_norm, self.disp) * (delta/delta_norm) 
        interpol_pose.translation = curr_pose.translation + delta
        # interpol_pose = goal_pose
        timestamp = rospy.Time.now().to_time() - self.init_time
            
        if not self.is_valid_pose(interpol_pose):
            print("Invalid Pose:", interpol_pose.translation)
            interpol_pose = curr_pose
        else:
            self.last_valid_pose_time = time.time()

        # account for tool offset
        if np.any(np.isnan(interpol_pose.translation)):
            interpol_pose = curr_pose
        # interpol_pose.translation[0] = self.START_POSE.translation[0]           # servoing
        interpol_pose = interpol_pose * self.TOOL_DELTA_POSE.inverse()

        traj_gen_proto_msg = PosePositionSensorMessage(
            id=self.id,
            timestamp=timestamp,
            position=interpol_pose.translation,
            quaternion=self.START_POSE.quaternion,
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION
            )
        )
        if np.any(np.isnan(interpol_pose.translation)):
            print("Nans in message")
        self.pub.publish(ros_msg)
        self.id += 1

        # print("Current error: ", np.linalg.norm(curr_pose.translation - goal_pose.translation))
        # print("Interpol pose: ", interpol_pose.translation)
        # print("Delta: ", delta)
        if (time.time() - self.last_valid_pose_time > self.timeout):
            self.is_resetting = True
            print('Terminating...')
            self.terminate_dynamic()
            print("Terminated")
            self.reset()

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
