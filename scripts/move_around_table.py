import actionlib
import rospy
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyRequest
import numpy as np
import subprocess
from tf.transformations import quaternion_about_axis, quaternion_multiply
import os

from table_extractor.msg import Table
from mongodb_store.message_store import MessageStoreProxy

from stare_at_tables.msg import StareAtTablesAction

class StareAtTables:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('stare_at_tables', StareAtTablesAction, self.execute, False)
		self.server.start()
		self.robot = Robot()
		self.base = self.robot.try_get('omni_base')
		self.whole_body = self.robot.try_get('whole_body')
		self.msg_store = MessageStoreProxy()
		self.rosbag_path = '/media/v4r/FF64-D891/tidy_up_pipeline/stare_at_tables'


	def execute(self, goal):
		for msg, meta in self.msg_store.query(Table._type):
			if msg.id==goal.id:
				poses = msg.viewposes
				for pose in poses:
					q = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
					q_rotate = quaternion_about_axis(np.pi/2, (0,0,1))
					q = quaternion_multiply(q_rotate, q)
					pose.pose.orientation.x = q[0]
					pose.pose.orientation.y = q[1]
					pose.pose.orientation.z = q[2]
					pose.pose.orientation.w = q[3]
				gaze_point = np.asarray([msg.center.point.x, msg.center.point.y, msg.center.point.z])
				height = msg.height
		poses = poses[::-1]

		self.whole_body.move_to_neutral()
		if height>0.4:
			self.whole_body.move_to_joint_positions({'arm_lift_joint': height-0.4})
		self.whole_body.move_to_joint_positions({'arm_flex_joint': -0.2,
					'arm_roll_joint': 1.570,
					'hand_motor_joint': 1.0,
					'head_pan_joint': 0.0,
					'head_tilt_joint': -0.75,
					'wrist_flex_joint': -1.57,
					'wrist_roll_joint': 0.0})
		self.whole_body.gaze_point(point=gaze_point, ref_frame_id='map')

		print(poses)
		stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
		stop_client.call(EmptyRequest())

		move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

		# wait for the action server to establish connection
		move_client.wait_for_server()

		# start rosbag recording
		rosbag_filename = os.path.join(self.rosbag_path, str(goal.id)+'.bag')
		cmd_rosbag = ['rosbag', 'record','-b','0','-O', rosbag_filename,'/hsrb/head_rgbd_sensor/rgb/image_raw', '/hsrb/head_rgbd_sensor/depth_registered/image_raw','/tf','/tf_static','/hsrb/head_rgbd_sensor/rgb/camera_info',"__name:=my_bag"]
		rosbagflag = False
		# move to poses
		for pose in poses:
			print(pose)
			goal = MoveBaseGoal()
			goal.target_pose = pose
			move_client.send_goal(goal)
			print('pose goal sent')
			self.whole_body.gaze_point(point=gaze_point, ref_frame_id='map')
			rospy.sleep(0.5)
			while True:
				self.whole_body.gaze_point(point=gaze_point, ref_frame_id='map')
				rospy.sleep(0.5)
				#print('gaze in loop')
				#print(move_client.get_goal_status_text())
				#print(move_client.get_state())
				if move_client.get_state()>1:
					if move_client.get_state()==3 and not rosbagflag:
						rosbag = subprocess.Popen(cmd_rosbag,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
						print("Start recording rosbag file...")
						rosbagflag = True
						rospy.sleep(0.5)
					break

		# kill rosbag
		if rosbagflag:
			rosbag.kill()
			subprocess.call(["rosnode", "kill", "/my_bag"])
		self.server.set_succeeded()
if __name__ == '__main__':
  rospy.init_node('stare_at_tables')
  server = StareAtTables()
  print('StareAtTables Action Server is ready')
  rospy.spin()
