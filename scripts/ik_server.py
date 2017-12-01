#!/usr/bin/env python

from math import *
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Pose
# Declare service files in CMakeLists.txt
from al5d.srv import *

def handle_ik(req):
  #rospy.loginfo(rospy.get_time())
  a1 = 0.071    # hoogte body
  a2 = 0.145  # lengte bovenarm
  a3 = 0.186 # lengte onderarm
  a4 = 0.115    # lengte pols en gripper
  arm_base_link = 'al5d_base_link'

  px = req.target.position.x
  py = req.target.position.y
  pz = req.target.position.z
  pitch = req.gripper_pitch
  roll = req.gripper_roll

  try:
    theta1 = atan2(py,px)
    theta3 = asin( ((cos(theta1)*px+sin(theta1)*py-cos(pitch)*a4)**2+(pz-a1-sin(pitch)*a4)**2-a2**2-a3**2)/(2*a2*a3) )
    n = (a3*sin(theta3) + a2)**2 + (cos(theta3)*a3)**2
    t = (cos(theta1)*px+sin(theta1)*py-cos(pitch)*a4)
    theta2 = asin( (-t * (a3*sin(theta3)+a2) + (pz-a1-sin(pitch)*a4) * cos(theta3)*a3) / n )
    theta4 = pitch - theta2 - theta3
    #print(theta1,theta2,theta3,theta4)
    #rospy.loginfo(rospy.get_time())
    return IkResponse(theta1,theta2,theta3,theta4, roll)
    
  except ValueError:
    return None

def ik_server():
  rospy.init_node('ik_server')
  s = rospy.Service('al5d_ik', Ik, handle_ik)
  print "Ready to do IK."
  rospy.spin()

if __name__ == "__main__":
  try:
    ik_server()
  except rospy.ROSInterruptException:
    pass
