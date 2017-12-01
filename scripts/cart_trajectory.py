#!/usr/bin/env python

"""
    cart_trajectory.py - Version 0.1
    
"""

import rospy
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from al5d.srv import Ik

class CartTrajectoryDemo():
    def __init__(self):
        rospy.init_node('joint_trajectory', anonymous=False)

        # Frame id and joints
        frame_id = "base"
        arm_joints = ['base_rotate',
                      'shoulder_tilt',
                      'elbow_tilt', 
                      'wrist_tilt',
                      'wrist_rotate',
                      'open_gripper']
        
        self.pos = Pose()
        self.pos.position.x = 0.0
        self.pos.position.y = 0.2
        self.pos.position.z = 0.1
        pitch = -1.45
        roll = 0.0
        joint_pos  = [0, 0, 0, 0, 0, 0]
 
        # Publisher to control the robot's joints
        self.command = rospy.Publisher('/joint_controller/command', JointTrajectory, queue_size=1)    

        rospy.wait_for_service("al5d_ik", 3)
        self.IkService = rospy.ServiceProxy("al5d_ik", Ik)
        response = self.IkService(self.pos, pitch, roll)
        joint_pos[0] = response.j1
        joint_pos[1] = response.j2
        joint_pos[2] = response.j3
        joint_pos[3] = response.j4
        joint_pos[4] = response.j5
     
        # Create a single-point arm trajectory with the joint_pos as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = joint_pos
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(2.0)
        arm_trajectory.header.frame_id = frame_id
        arm_trajectory.header.stamp = rospy.Time.now()

        # Publish trajectry
        rospy.sleep(0.5)
        self.command.publish(arm_trajectory)

        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        CartTrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
