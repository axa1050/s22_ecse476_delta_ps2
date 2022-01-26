# stdr_control
A reactive commander that accomplishes wall following using a robot with lidar.
It subscribes to front and left lidar boxes, and is a client of 'stdr_rotation_service'
It turns the robot CCW 90 degrees to face north, moves forward until obstacle in front or no wall on left,
halts if obstacle in front and turns CW 90 degrees, halt if wall not on left and turn CCW 90 degrees 
and move forward until wall appears on left again

## Example usage
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
to start the simulator.  Run a reactive command sequence with:
`rosrun stdr_nodes heading_seice`
`rosrun stdr_control reactive_commander`

## Directory structure
```
.
├── CMakeLists.txt
├── package.xml
├── README.md
└── src
    └── reactive_commander.cpp  The code for running wall following
```

    
