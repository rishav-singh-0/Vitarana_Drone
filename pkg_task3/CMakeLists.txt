cmake_minimum_required(VERSION 3.0.2)
project(pkg_task3)


add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy  
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs
  tf 
  tf2_ros
  std_msgs
  gazebo_ros
)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   prop_speed.msg
   edrone_cmd.msg
   MarkerData.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs  
 )


###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES 
	DEPENDS 
	    roscpp 
	    gazebo_ros
      std_msgs 
      geometry_msgs 
      tf
      message_runtime
      dynamic_reconfigure

)



