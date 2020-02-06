hello


#Terminal 3\
cd ~/catkin_ws\
catkin_make\
#Terminal 1\
roslaunch turtlebot_bringup minimal.launch\
#Terminal 2\
roslaunch mie443_contest1 gmapping.launch\
#Terminal 3\
rosrun mie443_contest1 contest1\

roslaunch mie443_stage turtlebot_in_stage_contest1.launch
