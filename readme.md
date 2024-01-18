# Deps 

ros2-humble

# Build

`$ colcon build --symlink-install`

# Lab 1

[turtletest](src%2Fturtletest)

`$ ros2 launch turtletest turtletest.launch.py`

* Use arrows to control the turtle
* Press Space to draw rectangles

# Lab 2 & 3

[mapping](src%2Fmapping)

`$ ros2 launch mapping navigation.launch.py`

* Use arrows to move the robot
* Press 0 to control robot by keys (default state)
* Press 1 to follow the predefined point
* Press 2 to move by radius
* Press Esc to move to the original position

Lab3 
* Press 3 to move nearby the wall

# Lab 4

[autorace](src%2Fautorace)

## Autorace 
`$ ros2 launch autorace autorace.launch.py`

* Use arrows to move the robot
* Press 1 to run autorace

## Red Ball detection

`$ ros2 launch autorace tracker.launch.py`

* Use arrows to move the robot
