import actionlib
import rospy
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyRequest
import numpy as np
import subprocess
from tf.transformations import quaternion_about_axis, quaternion_multiply

from table_extractor.msg import Table
from mongodb_store.message_store import MessageStoreProxy

robot = Robot()
whole_body = robot.get('whole_body')
base = robot.get('omni_base')



msg_store = MessageStoreProxy()
poses = []

##TODO select the plane
for msg, meta in msg_store.query(Table._type):
	if msg.id==1:
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

whole_body.move_to_neutral()
if height>0.4:
	whole_body.move_to_joint_positions({'arm_lift_joint': height-0.4})
whole_body.move_to_joint_positions({'arm_flex_joint': -0.2,
            'arm_roll_joint': 1.570,
            'hand_motor_joint': 1.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': -0.75,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0})
whole_body.gaze_point(point=gaze_point, ref_frame_id='map')

print(poses)
stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
stop_client.call(EmptyRequest())

move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

# wait for the action server to establish connection
move_client.wait_for_server()

# start rosbag recording
cmd_rosbag = ['rosbag', 'record','-b','0','/hsrb/head_rgbd_sensor/rgb/image_raw', '/hsrb/head_rgbd_sensor/depth_registered/image_raw','/tf','/tf_static','/hsrb/head_rgbd_sensor/rgb/camera_info',"__name:=my_bag"]
rosbagflag = False
# move to poses
for pose in poses:
	print(pose)
	goal = MoveBaseGoal()
	goal.target_pose = pose
	move_client.send_goal(goal)
    print('pose goal sent')
    whole_body.gaze_point(point=gaze_point, ref_frame_id='map')
    rospy.sleep(0.5)
	while True:
		whole_body.gaze_point(point=gaze_point, ref_frame_id='map')
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
rosbag.kill()
subprocess.call(["rosnode", "kill", "/my_bag"])
