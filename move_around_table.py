import actionlib
import rospy
import tf
import tf.transformations
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyRequest
import numpy as np
import subprocess

def fill_goal(goal_x, goal_y, goal_yaw):
	# fill ROS message	
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	pose.pose.position = Point(goal_x, goal_y, 0)
	quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
	pose.pose.orientation = Quaternion(*quat)

	goal = MoveBaseGoal()
	goal.target_pose = pose
	return goal

robot = Robot()
whole_body = robot.get('whole_body')
base = robot.get('omni_base')

whole_body.move_to_joint_positions({'head_tilt_joint': -1.0})
whole_body.gaze_point(point=[0.68, 0 ,0.45], ref_frame_id='map')

poses = np.loadtxt('sasha_lab.txt', delimiter=',')


stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
stop_client.call(EmptyRequest())

move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

# wait for the action server to establish connection
move_client.wait_for_server()

# start rosbag recording
print("Start recording rosbag file...")
cmd_rosbag = ['rosbag', 'record','-b','0','/hsrb/head_rgbd_sensor/rgb/image_raw', '/hsrb/head_rgbd_sensor/depth_registered/image_raw','/tf','/tf_static','/hsrb/head_rgbd_sensor/rgb/camera_info',"__name:=my_bag"]
rosbag = subprocess.Popen(cmd_rosbag,stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# move to poses
for pose in poses:
	print(pose)
	goal = fill_goal(*pose)
	move_client.send_goal(goal)
	while True:
		whole_body.gaze_point(point=[0.68, 0 ,0.45], ref_frame_id='map')
		print('gaze in loop')
		if move_client.get_state()>1:
			break

# kill rosbag
rosbag.kill()
subprocess.call(["rosnode", "kill", "/my_bag"])