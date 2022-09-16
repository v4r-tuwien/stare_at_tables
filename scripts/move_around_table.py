#!/usr/bin/python2
import actionlib
import rospy
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyRequest
import numpy as np
import subprocess
from tf.transformations import quaternion_about_axis, quaternion_multiply
import os

from edith_msgs.msg import Table
from mongodb_store.message_store import MessageStoreProxy

from stare_at_tables.msg import StareAtTablesAction

import time


class StareAtTables:
	def __init__(self):
		self.server = actionlib.SimpleActionServer(
            'stare_at_tables', StareAtTablesAction, self.execute, False)
		self.server.start()
		self.robot = Robot()
		self.base = self.robot.try_get('omni_base')
		self.whole_body = self.robot.try_get('whole_body')
		self.msg_store = MessageStoreProxy()
		self.rosbag_path = rospy.get_param(
            '/stare_at_tables/rosbag_path', '/home/v4r/Markus_L/sasha_lab_bag')
		if not os.path.exists(self.rosbag_path):
			os.makedirs(self.rosbag_path)
		self.storage_path = rospy.get_param(
            '/stare_at_tables/storage_path', 'v4r@rocco:/media/v4r/Sasha_external/tidy_up_pipeline/stare_at_tables')

		cmd_dir = ['ssh', 'v4r@rocco',
                   'mkdir -p /media/v4r/Sasha_external/tidy_up_pipeline/stare_at_tables']
		directory = subprocess.Popen(
            cmd_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		directory.wait()
		self.topics = rospy.get_param('/stare_at_tables/topics', ['/hsrb/head_rgbd_sensor/rgb/image_rect_color',
                                      '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', '/tf', '/tf_static', '/hsrb/head_rgbd_sensor/rgb/camera_info'])

	def execute(self, goal):
		print(goal)
		for msg, meta in self.msg_store.query(Table._type):
			if msg.id == goal.id:
				poses = msg.viewposes
				for pose in poses:
					q = [pose.pose.orientation.x, pose.pose.orientation.y,
						pose.pose.orientation.z, pose.pose.orientation.w]
					q_rotate = quaternion_about_axis(np.pi/2, (0, 0, 1))
					q = quaternion_multiply(q_rotate, q)
					pose.pose.orientation.x = q[0]
					pose.pose.orientation.y = q[1]
					pose.pose.orientation.z = q[2]
					pose.pose.orientation.w = q[3]
				gaze_point = np.asarray(
                	[msg.center.point.x, msg.center.point.y, msg.center.point.z])
				height = msg.height
		poses = poses[::-1]

		self.whole_body.move_to_neutral()
		if height > 0.4:
			self.whole_body.move_to_joint_positions(
				{'arm_lift_joint': height-0.4})
		self.whole_body.move_to_joint_positions({'arm_flex_joint': -0.2,
                                                 'arm_roll_joint': 1.570,
                                                 'hand_motor_joint': 1.0,
                                                 'head_pan_joint': 0.0,
                                                 'head_tilt_joint': -0.75,
                                                 'wrist_flex_joint': -1.57,
                                                 'wrist_roll_joint': 0.0})
		print(gaze_point)
		self.whole_body.gaze_point(point=gaze_point, ref_frame_id='map')

		stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
		stop_client.call(EmptyRequest())

		move_client = actionlib.SimpleActionClient(
			'/move_base/move', MoveBaseAction)

        # wait for the action server to establish connection
		move_client.wait_for_server()

        # start rosbag recording
		rosbag_filename = os.path.join(self.rosbag_path, str(goal.id)+'.bag')
		print(rosbag_filename)
        #cmd_rosbag = ['rosbag', 'record','-b','0','-O', rosbag_filename,'/hsrb/head_rgbd_sensor/rgb/image_raw', '/hsrb/head_rgbd_sensor/depth_registered/image_raw','/tf','/tf_static','/hsrb/head_rgbd_sensor/rgb/camera_info',"__name:=my_bag"]
		cmd_rosbag = ['rosbag', 'record', '-b', '0', '-O', rosbag_filename]
		cmd_rosbag.extend(self.topics)
		print(self.topics)
		cmd_rosbag.append("__name:=my_bag")
		rosbagflag = False
		# move to poses
		for pose in poses:
			goal = MoveBaseGoal()
			goal.target_pose = pose
			move_client.send_goal(goal)
			print(gaze_point)
			self.whole_body.gaze_point(point=gaze_point, ref_frame_id='map')
			rospy.sleep(0.5)
			while True:
				print(gaze_point)
				self.whole_body.gaze_point(
					point=gaze_point, ref_frame_id='map')
				rospy.sleep(0.5)
				#print('gaze in loop')
				# print(move_client.get_goal_status_text())
				# print(move_client.get_state())
				if move_client.get_state() > 1:
					if move_client.get_state() == 3 and not rosbagflag:
						rosbag = subprocess.Popen(
							cmd_rosbag, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
						print("Start recording rosbag file...")
						rosbagflag = True
						rospy.sleep(3.5)
					break

		# kill rosbag
		if rosbagflag:
			rospy.sleep(3.5)
			rosbag.kill()
			subprocess.call(["rosnode", "kill", "/my_bag"])
			start = time.time()
			cmd_move = ['scp', rosbag_filename, self.storage_path]
			move = subprocess.Popen(
				cmd_move, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			move.wait()
			cmd_remove = ['rm', rosbag_filename]
			remove = subprocess.Popen(
				cmd_remove, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			remove.wait()
			end = time.time()

			print(end - start)
			self.server.set_succeeded()
		else:
			self.server.set_aborted()


if __name__ == '__main__':
	rospy.init_node('stare_at_tables')
	server = StareAtTables()
	print('StareAtTables Action Server is ready')
	rospy.spin()
