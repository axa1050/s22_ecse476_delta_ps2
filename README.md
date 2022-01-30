# s22_ecse476_delta_ps2

THis is a over all read me file which talks about the project. 
The project is intended to combine 3 services to make a robot wall following.
Note: This is done on ROS Noetic.
Usage:
roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch 
rosrun front_lidar_alarm front_lidar_alarm
rosrun left_lidar_alarm left_lidar_alarm
rosrun stdr_nodes heading_service
rosrun stdr_control reactive_commander


