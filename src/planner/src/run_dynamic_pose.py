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
        translation=np.array([0.4, 0, 0.4]),
        rotation=np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]]),
        from_frame="franka_tool",
        to_frame="world",
    )

def moveToCatch(fa, goal_pose, velocity=0.25, dt=0.02):
    start = time.time()
    curr_pose = fa.get_pose()
    delta_trans = np.linalg.norm(goal_pose.translation - curr_pose.translation)
    T = float(delta_trans / velocity)
    ts = np.arange(0, T, dt)

    # weights and pose_traj calculations take few 100ms each
    # Time taken is proportional to dt (or) length of the discretized trajectory
    weights = [min_jerk_weight(t, T) for t in ts]
    pose_traj = [curr_pose.interpolate_with(goal_pose, w) for w in weights]

    rospy.loginfo("Initializing Sensor Publisher")
    pub = rospy.Publisher(
        FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000
    )
    rate = rospy.Rate(1 / dt)

    rospy.loginfo(f"Trajectory time: {T:0.4f}s.\tPublishing pose trajectory...")
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed

    fa.goto_pose(pose_traj[1], duration=dt, dynamic=True, buffer_time=5 + 2 * T, block=False)
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
    fa.wait_for_skill()
    # while(np.linalg.norm(fa.get_joint_velocities()) > 1e-2):
    #     pass
    rospy.loginfo(f"Movement Done. Runtime: {time.time() - start:0.4f}s")


if __name__ == "__main__":

    # initialize arm
    fa = FrankaArm()
    # Commands Arm to go to default hardcoded home joint configuration
    # fa.reset_joints()

    # Go to start position
    fa.goto_pose(START_POSE, duration=5)

    # Create a displacement w.r.to the tool and create a goal pose
    curr_pose = fa.get_pose()
    T_delta = RigidTransform(
        translation=np.array([0.00, 0.2, 0]),
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)),
        from_frame=curr_pose.from_frame,
        to_frame=curr_pose.from_frame,
    )
    goal_pose = curr_pose * T_delta

    moveToCatch(fa, goal_pose, velocity=0.2, dt=0.01)
