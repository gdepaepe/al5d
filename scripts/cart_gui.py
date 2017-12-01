#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from al5d.srv import Ik
from Tkinter import *
import ttk



class CartGUI():
    def __init__(self):
        rospy.init_node('cart_gui', anonymous=False)
        rospy.wait_for_service("al5d_ik", 3)
        self.IkService = rospy.ServiceProxy("al5d_ik", Ik)
        self.command = rospy.Publisher('/joint_controller/command', JointTrajectory, queue_size=1)

        root = Tk()
        root.title("Al5d Robot")

        mainframe = ttk.Frame(root, padding="3 3 12 12")
        mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        mainframe.columnconfigure(0, weight=1)
        mainframe.rowconfigure(0, weight=1)

        self.x = StringVar()
        self.x.set("0.2")
        self.y = StringVar()
        self.y.set("0.0")
        self.z = StringVar()
        self.z.set("0.1")
        self.pitch = StringVar()
        self.pitch.set("-1.45")
        self.roll = StringVar()
        self.roll.set("0.0")
        self.gripper = StringVar()
        self.gripper.set("0.0")

        x_entry = ttk.Entry(mainframe, width=7, textvariable=self.x)
        x_entry.grid(column=2, row=1, sticky=(W, E))
        y_entry = ttk.Entry(mainframe, width=7, textvariable=self.y)
        y_entry.grid(column=2, row=2, sticky=(W, E))
        z_entry = ttk.Entry(mainframe, width=7, textvariable=self.z)
        z_entry.grid(column=2, row=3, sticky=(W, E))
        pitch_entry = ttk.Entry(mainframe, width=7, textvariable=self.pitch)
        pitch_entry.grid(column=2, row=4, sticky=(W, E))
        roll_entry = ttk.Entry(mainframe, width=7, textvariable=self.roll)
        roll_entry.grid(column=2, row=5, sticky=(W, E))
        gripper_entry = ttk.Entry(mainframe, width=7, textvariable=self.gripper)
        gripper_entry.grid(column=2, row=6, sticky=(W, E))

        ttk.Button(mainframe, text="GO", command=self.move).grid(column=2, row=7, sticky=W)

        ttk.Label(mainframe, text="X").grid(column=1, row=1, sticky=E)
        ttk.Label(mainframe, text="Y").grid(column=1, row=2, sticky=E)
        ttk.Label(mainframe, text="Z").grid(column=1, row=3, sticky=E)
        ttk.Label(mainframe, text="Pitch").grid(column=1, row=4, sticky=E)
        ttk.Label(mainframe, text="Roll").grid(column=1, row=5, sticky=E)
        ttk.Label(mainframe, text="Gripper").grid(column=1, row=6, sticky=E)

        for child in mainframe.winfo_children():
            child.grid_configure(padx=5, pady=5)

        x_entry.focus()
        root.bind('<Return>', self.move)
        root.bind('<KP_Enter>', self.move)

        root.mainloop()

    def move(self, *args):
        try:
            # Frame id and joints
            frame_id = "base"
            arm_joints = ['base_rotate',
                          'shoulder_tilt',
                          'elbow_tilt',
                          'wrist_tilt',
                          'wrist_rotate',
                          'open_gripper']

            pos = Pose()
            pos.position.x = float(self.x.get())
            pos.position.y = float(self.y.get())
            pos.position.z = float(self.z.get())
            pitch = float(self.pitch.get())
            roll = float(self.roll.get())
            joint_pos = [0, 0, 0, 0, 0, 0]

            response = self.IkService(pos, pitch, roll)
            joint_pos[0] = response.j1
            joint_pos[1] = response.j2
            joint_pos[2] = response.j3
            joint_pos[3] = response.j4
            joint_pos[4] = response.j5
            joint_pos[5] = float(self.gripper.get())

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
        except ValueError:
            pass


if __name__ == '__main__':
    try:
        CartGUI()
    except rospy.ROSInterruptException:
        pass
    
