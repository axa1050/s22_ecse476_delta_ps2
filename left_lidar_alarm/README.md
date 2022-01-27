# left_lidar_alarm

This code subscribes to topic `robot0/laser_0`, which is published by the Simple 2-D Robot simulator. 
The signal interpretation looks at the lasers on the left side of the robot and checks a rectangular box for obstacles. If a ping distance is measured less than the rectangle's boundaries, the lidar listener publishes a warning signal on topic `left_lidar_alarm`. 

Description of the rectangle region: From the maximum angle to the y-symmetry of the angle, resulting in 120degrees to 60degrees wrt straight front direction. The rectangles far edge is 0.8m away form the center of the robot. The width of the rectanlge is 0.3m.

## Example usage
Start up the STDR simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start the lidar alarm node:
`rosrun left_lidar_alarm left_lidar_alarm`

## Directory structure
```
.
├── CMakeLists.txt
├── package.xml
├── README.md
└── src
    └── left\_lidar\_alarm.cpp  The code for the left side lidar alarm
```

