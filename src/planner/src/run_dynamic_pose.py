#!/usr/bin/env python

import time
import numpy as np
import rospy

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage

from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import min_jerk_weight

# A good pose from where robot can start moving from
START_POSE = RigidTransform(
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

TOOL_DELTA_POSE = RigidTransform(
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

BOX_CORNER_MIN = np.array([0.4, -0.3, 0.2])
BOX_CORNER_MAX = np.array([0.5, 0.3, 0.45])


def is_valid_pose(pose):
    return np.all(pose.translation <= BOX_CORNER_MAX) and np.all(pose.translation >= BOX_CORNER_MIN)


def moveToCatch(fa, goal_pose, disp=0.15, dt=0.02):
    curr_pose = fa.get_pose()

    # weights and pose_traj calculations take few 100ms each
    # Time taken is proportional to dt (or) length of the discretized trajectory
    # weights = [min_jerk_weight(t, T) for t in ts]
    # pose_traj = [curr_pose.interpolate_with(goal_pose, w) for w in weights]

    rospy.loginfo("Initializing Sensor Publisher")
    pub = rospy.Publisher(
        FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000
    )
    rate = rospy.Rate(1 / dt)

    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed

    fa.goto_pose(curr_pose, duration=dt, dynamic=True, buffer_time=5, block=False)
    start = time.time()
    interpol_pose = curr_pose
    i = 0
    init_time = rospy.Time.now().to_time()
    while(np.linalg.norm(curr_pose.translation - goal_pose.translation) > 1e-2):
        curr_pose = fa.get_pose()
        delta = goal_pose.translation - curr_pose.translation
        delta_norm = np.linalg.norm(delta)
        delta = min(delta_norm, disp) * (delta/delta_norm) 
        interpol_pose.translation = curr_pose.translation + delta
        timestamp = rospy.Time.now().to_time() - init_time
        
        # if min(delta_norm, disp)==0.15:
        #     print("limiting speed")

        try: 
            assert is_valid_pose(interpol_pose)
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

        # rospy.loginfo("Publishing: ID {}".format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()
        i = i + 1

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
    pub.publish(ros_msg)
    fa.wait_for_skill()
    # while(np.linalg.norm(fa.get_joint_velocities()) > 1e-2):
    #     pass
    rospy.loginfo(f"Movement Done. Runtime: {time.time() - start:0.4f}s")


if __name__ == "__main__":

    # initialize arm
    fa = FrankaArm(with_gripper=False)
    # Commands Arm to go to default hardcoded home joint configuration
    # fa.reset_joints()


    # print(fa.get_tool_base_pose())
    # Go to start position
    fa.set_tool_delta_pose(TOOL_DELTA_POSE)
    fa.goto_pose(START_POSE, duration=5)

    # # Create a displacement w.r.to the tool and create a goal pose
    # curr_pose = fa.get_pose()
    # T_delta = RigidTransform(
    #     translation=np.array([0.0, -0.275, 0]),
    #     from_frame=curr_pose.to_frame,
    #     to_frame=curr_pose.to_frame,
    # )
    # goal_pose = T_delta * curr_pose

    # moveToCatch(fa, goal_pose, disp=0.15, dt=0.01)
