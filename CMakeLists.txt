cmake_minimum_required(VERSION 2.8.3)
project(stare_at_tables)

find_package(catkin REQUIRED 
genmsg 
rospy
std_msgs
actionlib_msgs
move_base_msgs
hsrb_interface_py
table_extractor)

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES
   StareAtTables.action
 )
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  
   actionlib_msgs
   move_base_msgs
 )

catkin_package()
include_directories()

