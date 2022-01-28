# lidar_alarm

A simple, illustrative program to show how to subscribe to a LIDAR signal and interpret the signal.
This code subscribes to topic `robot0/laser_0`, which is published by the Simple 2-D Robot simulator.
This signal looks the min_safe_distance of 0.5 m and with a box width of 0.2m. calculates the distance of the robot 
in the environment with the distance recorded in the laser , compares the value if the robot is closer to the obstacle
it raises a warnining , if not it keep on checking until the program is terminated.

If this ping distance is less than some danger threshold, the lidar listener publishers a warning signal on
topic `front_lidar_alarm`.  The distance of the forward ping is also published, on topic `lidar_dist`.



## Example usage
Start up the STDR simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start the lidar alarm node:
 `rosrun front_lidar_alarm front_lidar_alarm`
 Have the controller code monitor the `front_lidar_alarm` topic and do something intelligent with the information.

 
  ## Directory structure
```
.
├── CMakeLists.txt
├── package.xml
├── README.md
└── src
    └── front_lidar_alarm.cpp  The code for the front side lidar alarm
    
    
