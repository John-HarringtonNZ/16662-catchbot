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


def moveToCatch(fa, goal_pose, velocity=0.25, dt=0.02):
    start = time.time()
    curr_pose = fa.get_pose()
    delta_trans = np.linalg.norm(goal_pose.translation - curr_pose.translation)
    T = float(delta_trans / velocity)
    ts = np.arange(0, T, dt)

    weights = [min_jerk_weight(t, T) for t in ts]
    pose_traj = [curr_pose.interpolate_with(goal_pose, w) for w in weights]

    rospy.loginfo("Initializing Sensor Publisher")
    pub = rospy.Publisher(
        FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000
    )
    rate = rospy.Rate(1 / dt)

    rospy.loginfo(f"Trajectory time: {T:0.4f}s.\tPublishing pose trajectory...")
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(pose_traj[1], duration=dt, dynamic=True, buffer_time=5 + 2 * T)
    init_time = rospy.Time.now().to_time()
    for i in range(2, len(ts)):
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i,
            timestamp=timestamp,
            position=pose_traj[i].translation,
            quaternion=pose_traj[i].quaternion,
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION
            )
        )

        # rospy.loginfo("Publishing: ID {}".format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()

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
    rospy.loginfo(f"Movement Done. Runtime: {time.time() - start:0.4f}s")


if __name__ == "__main__":

    # initialize arm
    fa = FrankaArm()
    fa.reset_joints()

    # Go to start position
    curr_pose = fa.get_pose()
    home_pose = curr_pose.copy()
    T_delta = RigidTransform(
        translation=np.array([0.1, 0, 0.1]),  # change this to be good start position
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)),
        from_frame=curr_pose.from_frame,
        to_frame=curr_pose.from_frame,
    )
    home_pose = curr_pose * T_delta
    fa.goto_pose(home_pose, duration=5)

    T_delta_2 = RigidTransform(
        translation=np.array([0.00, 0.4, 0]),  # change this to be good start position
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)),
        from_frame=home_pose.from_frame,
        to_frame=home_pose.from_frame,
    )
    goal_pose = home_pose * T_delta_2

    moveToCatch(fa, goal_pose, velocity=0.75, dt=0.01)
