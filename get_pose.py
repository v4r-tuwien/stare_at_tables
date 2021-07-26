import os,sys
import math
import actionlib
import control_msgs.msg
import geometry_msgs.msg
import rospy
import std_srvs.srv
import trajectory_msgs.msg

import hsrb_interface
from hsrb_interface import geometry
from geometry_msgs.msg import Pose


robot = hsrb_interface.Robot()
whole_body = robot.get("whole_body")
omni_base = robot.get("omni_base")
gripper = robot.get("gripper")

target_fn = sys.argv[1]
while True:
	current_pose = omni_base.pose
	print(current_pose)

	if(os.path.exists(target_fn)):
		f=open(target_fn,"a+")
	else:
		f=open(target_fn,"w+")
	f.write("{}, {}, {}\n".format(current_pose[0],current_pose[1],current_pose[2]))
	f.close()
	raw_input("Press Enter to add a new pose")

robot.close()
