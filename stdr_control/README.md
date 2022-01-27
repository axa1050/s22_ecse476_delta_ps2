# stdr_control
A reactive commander that accomplishes wall following using a robot with lidar.
It subscribes to front and left lidar boxes, and is a client of 'stdr_rotation_service'
It turns the robot CCW 90 degrees to face north, moves forward until obstacle in front or no wall on left,
halts if obstacle in front and turns CW 90 degrees, halt if wall not on left and turn CCW 90 degrees 
and move forward until wall appears on left again

## Example usage
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
to start the simulator.  Run a reactive command sequence with:
`rosrun stdr_nodes heading_service`
`rosrun stdr_control reactive_commander`

Alternatively, run:
`roslaunch stdr_control prep_for_reactive_commander.launch`
`roslaunch stdr_control reactive_commander.launch`


## Directory structure
```
.
├── CMakeLists.txt
├── package.xml
├── launch
    └── prep\_for\_reactive_commander.launch  The launch file to prepare the other nodes before reactive commander
    └── reactive_commander.launch  The launch file for just runnning the reactive commander.
├── README.md
└── src
    └── reactive_commander.cpp  The code for running wall following
```

    
