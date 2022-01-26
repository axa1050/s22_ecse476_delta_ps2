# stdr_nodes

Build a heading controller for the stdr simulator
It contains a service that has the name 'stdr_rotation_service'
It should accept a desired rotation, calculate the desired heading, 
command the robot to rotate the desired amount, 
and verify successful completion.
Respond with the actual desired heading.

## Example usage
rosrun stdr_nodes heading_service

## Directory structure
```
.
├── CMakeLists.txt
├── include
├── package.xml
├── README.md
└── src
    ├── heading_service.cpp      The code for the heading service that accepts the desired rotation
    └── heading_test_client.cpp
```
    
