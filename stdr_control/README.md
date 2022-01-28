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


## Observations
- The shape of the region of obstacle check is important. The single lidar check is fast but not reliable or even sufficient for the task. The disk piece region can be sufficient but not reliable for smaller scale obstacles. The rectangle box implementation is both reliable and for our purposes sufficiently fast. 
- The scale of the obstacle check region is important. The scale both determines which obstacles can be detected as robot apporaches and how different lidar readings are handled together. On first point, it affects how tunnels or step shaped obstacles are seen. On second point, for example a long frontal detection region will detect the wall and the robot will rotate so that wall is on the left but a short side detection region will not recognize the wall since the wall is outside of its region.   


## Suggestions
- Package users should check the parameters and make adjustments on the rectangle region as needed in order to fit their use case.


## Possible future work
- [ ] Making nodes scopable so they can be used for different robots 
- [ ] Making nodes to be usable for multiple robots simultaneously
- [ ] Parametrize the nodes and their launch files for terminal outputs, publishing information options etc
