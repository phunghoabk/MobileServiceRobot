#@phunghoa
This robot is differential drive robot.
#kill gazebo
killall -9 gzserver

colcon build
#source gazebo env
. /usr/share/gazebo/setup.sh
. install/setup.bash 
ros2 launch dolly_gazebo dolly.launch.py # empty_world

# "anrobot_follow" package is node follwing human with laser scan. when human move, find the closet laser hit and run over to it.
ros2 run anrobot_follow anrobot_follow 

# "point_publisher" package is a node of publishing a point(x,y) for testing
ros2 run point_publisher point_publisher

# "following" package is the following node, the robot will allway follow the human position (x,y) and keep a distan d =1m. This algorithm is potential field.
ros2 run following following

#turtlebot cmd_vel style
ros2 run anrobot_teleop teleop_keyboard --ros-args -r __ns:=/anrobot

#seting cmd_vel then run style
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/anrobot



