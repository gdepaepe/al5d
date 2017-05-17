#!/usr/bin/env python

"""
    joint_trajectory.py - Version 0.1
    
"""

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryDemo():
    def __init__(self):
        rospy.init_node('joint_trajectory', anonymous=False)

        # Define positions
        joint_start = [0, 0, 0, 0, 0, 0]
        joint_pos1 = [-1, -0.2, -0.2, -1.1, -1, 1.5]
        joint_pos2 = [1, -0.2, -0.2, -1.1, 1, 0]
        joint_pos3 = [0, -0.8, 0.5, 0.5, 0, 0]

        # Create trajectory
        self.init_trajectory()
        self.add_trajectory_point(joint_start, joint_pos1)
        self.add_trajectory_point(joint_pos1,joint_pos2)
        self.add_trajectory_point(joint_pos2,joint_pos3)
        self.add_trajectory_point(joint_pos3,joint_pos3)
        self.add_trajectory_point(joint_pos3,joint_pos1)
        self.add_trajectory_point(joint_pos1,joint_pos1)

        # Publish trajectry
        rospy.sleep(1)
        self.command.publish(self.arm_trajectory)
        rospy.loginfo('...done')

    def init_trajectory(self):
        self.command = rospy.Publisher('/joint_controller/command', JointTrajectory, queue_size=1)
        frame_id = "base"
        arm_joints = ['base_rotate',
                      'shoulder_tilt',
                      'elbow_tilt',
                      'wrist_tilt',
                      'wrist_rotate',
                      'open_gripper']
        self.arm_trajectory = JointTrajectory()
        self.arm_trajectory.header.frame_id = frame_id
        self.arm_trajectory.header.stamp = rospy.Time.now()
        self.arm_trajectory.joint_names = arm_joints

    def add_trajectory_point(self, start, finish):
        delta = [m - n for m, n in zip(finish, start)]
        for i in range(50):
            p = JointTrajectoryPoint()
            p.positions = [m + n / 50.0 * i for m, n in zip(start, delta)]
            p.time_from_start = rospy.Duration.from_sec(0.01)
            self.arm_trajectory.points.append(p)


if __name__ == '__main__':
    try:
        JointTrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
