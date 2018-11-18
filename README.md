# Akupara

A door passing turtlebot for ROS

# Running
  roslaunch turtlebot_gazebo turtlebot_world.launch
  roslaunch trajectory_skeleton_456 view_robot.launch
  rosrun akupara openCV.py - Do not forget to chmod +x this file

  - If doesnt work try running as sudo (if installiation done as sudo)

  To display camera topic:

  rostopic echo /camera/rgb/image_raw
  rostopic info /camera/rgb/image_raw
  rosmsg show /camera/rgb/image_raw

# Building
  cd ~/catkin_ws && catkin_make

# Creating package
  cd ~/catkin/src
  catkin_create_pkg akupara OpenCV tf geometry_msgs std_msgs rospy roscpp
