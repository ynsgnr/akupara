# Akupara

A door passing turtlebot for ROS. This project implemented as BLG456E Course Project in 2018.

# Running

  To run the environment designed for this task:
  ~~~~
  roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=/home/$USER/catkin_ws/src/akupara/worlds/akupara_playground.world
  ~~~~
  If you are running as a different user (or you are SU) please change $USER with your user name
  
  To launch designed control module run
  ~~~~
  rosrun akupara openCV.py
  ~~~~
  If you run issues with running you might need to give permissions to `openCV.py` with `chmod +x` command on this file

  To publish color (change `red` with desired color):
  ~~~~
  rostopic pub door_color std_msgs/String red
  ~~~~

  Avaliable colors for now are:
  * blue
  * green
  * pink
  * yellow
  * red

# Building
  Copy project to ROS workspace and then run `catkin_make` on workspace directory

# Note
  - To drive robot around:
  ~~~~
  roslaunch turtlebot_teleop keyboard_teleop.launch
  ~~~~

  - To display camera topic:
  ~~~~
  rostopic echo /camera/rgb/image_raw
  rostopic info /camera/rgb/image_raw
  rosmsg show /camera/rgb/image_raw
  ~~~~
