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
    def __init__(self, fa) -> None:
        # initialize arm
        print('Initialized Arm')
        self.fa = fa
        self.TOOL_DELTA_POSE = RigidTransform(
        translation=np.array([0.106, -0.106, -0.01]),
        # rotation=np.array([[0.70710678, -0.70710678, 0], [-0.70710678, -0.70710678, 0], [0, 0, 1.0]]),
        # rotation=np.array([[7.07185286e-01, -7.07028188e-01, -3.36139057e-04],
        #                     [-7.07028245e-01, -7.07185062e-01, -5.90835436e-04],
        #                     [1.80024788e-04,  6.55489934e-04, -9.99999769e-01]]),
        rotation=np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0,  0, 1]]),
        from_frame="franka_tool",
        to_frame="franka_tool_base"
    )
        fa.set_tool_delta_pose(self.TOOL_DELTA_POSE)
        self.sub = rospy.Subscriber(
            "/intersect_position", 
            PointStamped,
            self.get_goal_position)
        self.pub = rospy.Publisher(
            FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000
        )
        print(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.moveToCatch)
        self.timeout = 5

        self.dt = 0.01
        self.rate = rospy.Rate(1 / self.dt)
        self.disp = 0.15


        # A good pose from where robot can start moving from

        self.START_POSE = RigidTransform(
                translation=np.array([0.45, 0, 0.4]),
                # rotation=np.array([[0.70710678, -0.70710678, 0], [-0.70710678, -0.70710678, 0], [0, 0, 1.0]]),
                # rotation=np.array([[7.07185286e-01, -7.07028188e-01, -3.36139057e-04],
                #                     [-7.07028245e-01, -7.07185062e-01, -5.90835436e-04],
                #                     [1.80024788e-04,  6.55489934e-04, -9.99999769e-01]]),
                rotation=np.array([[7.07185286e-01, -7.07028188e-01, -3.36139057e-04],
                                    [-7.07028245e-01, -7.07185062e-01, -5.90835436e-04],
                                    [1.80024788e-04,  6.55489934e-04, -9.99999769e-01]]),
                from_frame="franka_tool",
                to_frame="world"
            )
        self.goal_pose_msg = None

        self.BOX_CORNER_MIN = np.array([0.3, -0.3, 0.2])
        self.BOX_CORNER_MAX = np.array([0.5, 0.3, 0.55])

        self.reset()

    def reset(self):
        print('Resetting Arm')
        self.fa.goto_pose(self.START_POSE, duration=5)
        # self.goal_pose_msg = PointStamped()
        # self.goal_pose_msg.point.x = self.START_POSE.translation[0]
        # self.goal_pose_msg.point.y = self.START_POSE.translation[1]
        # self.goal_pose_msg.point.z = self.START_POSE.translation[2]
        self.goal_pose_msg = None
        print('Reset complete')

    def get_goal_position(self, goal_pose_msg):
        self.goal_pose_msg = goal_pose_msg
        
    def is_valid_pose(self, pose):
        return np.all(pose.translation <= self.BOX_CORNER_MAX) and np.all(pose.translation >= self.BOX_CORNER_MIN)


    def moveToCatch(self, event):
        self.fa.wait_for_skill()
        if self.goal_pose_msg is None:
            print('Waiting for goal_pose_msg')
            return

        # Check for timeout
        curr_time = rospy.Time.now().to_sec()
        last_msg_time = self.goal_pose_msg.header.stamp.to_sec()
        if (curr_time - last_msg_time >= self.timeout):
            self.reset()
            return

        curr_pose = self.fa.get_pose()
        start_time = time.time()
        self.fa.goto_pose(curr_pose, duration=self.dt, dynamic=True, buffer_time=5, block=False)
        print("Time", time.time()-start_time)
        goal_pose = RigidTransform(
            translation=np.array([
                self.goal_pose_msg.point.x,
                self.goal_pose_msg.point.y,
                self.goal_pose_msg.point.z
            ]),
            from_frame=curr_pose.to_frame,
            to_frame=curr_pose.to_frame,
        )
        # T_delta = RigidTransform(
        #     translation=np.array([0.0, -0.25, 0]),
        #     from_frame=curr_pose.to_frame,
        #     to_frame=curr_pose.to_frame,
        # )
        # goal_pose = T_delta * curr_pose
        print(np.linalg.norm(curr_pose.translation - goal_pose.translation))
        print(goal_pose.translation)
        print(curr_pose.translation)

        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        # self.fa.goto_pose(curr_pose, duration=self.dt, dynamic=True, buffer_time=5, block=False)
        start = time.time()
        interpol_pose = curr_pose
        i = 0
        init_time = rospy.Time.now().to_time()
        while(np.linalg.norm(curr_pose.translation - goal_pose.translation) > 5e-2):
            print("Current error: ", np.linalg.norm(curr_pose.translation - goal_pose.translation))
            loop_start_time = time.time()
            curr_pose = self.fa.get_pose()
            delta = goal_pose.translation - curr_pose.translation
            delta_norm = np.linalg.norm(delta)
            delta = min(delta_norm, self.disp) * (delta/delta_norm) 
            interpol_pose.translation = curr_pose.translation + delta
            # interpol_pose = goal_pose
            timestamp = rospy.Time.now().to_time() - init_time
            
            try: 
                assert self.is_valid_pose(interpol_pose)
            except AssertionError:
                print("Invalid Pose:", interpol_pose.translation)
                return

            traj_gen_proto_msg = PosePositionSensorMessage(
                id=i,
                timestamp=timestamp,
                position=interpol_pose.translation,
                quaternion=interpol_pose.quaternion,
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION
                )
            )
            print(len(ros_msg.trajectoryGeneratorSensorData.sensorData))

            # rospy.loginfo("Publishing: ID {}".format(traj_gen_proto_msg.id))
            print('Publishing msg!')
            self.pub.publish(ros_msg)
            # print(time.time() - loop_start_time)
            self.rate.sleep()
            i = i + 1
        print("Loop ended: ", time.time() - start)
        # Stop the skill
        # Alternatively can call fa.stop_skill()
        term_proto_msg = ShouldTerminateSensorMessage(
            timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True
        )
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE
            )
        )
        self.pub.publish(ros_msg)
        self.fa.wait_for_skill()
        print("Skill Done: ", time.time() - start)
        rospy.loginfo(f"Movement Done. Runtime: {time.time() - start:0.4f}s")


if __name__ == "__main__":
    fa = FrankaArm(with_gripper=False)

    # rospy.init_node('planning_node', disable_signals=True, log_level=rospy.INFO)
    PlanningNode(fa)
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
