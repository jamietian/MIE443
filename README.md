hello


#Terminal 4\
cd ~/catkin_ws\
catkin_make\
#Terminal 1\
#below starts the UI\
roslaunch mie443_stage turtlebot_in_stage_contest1.launch\
#below line simulates laser scanning\
rosrun gmapping slam_gmapping\
#Terminal 2\
roslaunch turtlebot_bringup minimal.launch\
#Terminal 3\
roslaunch mie443_contest1 gmapping.launch\
#Terminal 4\
rosrun mie443_contest1 contest1


#mapping\
roslaunchturtlebot_rviz_launchersview_navigation.launch
save map\
rosrunmap_servermap_saver–f your_map_name\
#Saves the gmappingmap to the current directory with your own file name
