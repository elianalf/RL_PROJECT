# ABB IRB 6640 Gazebo

##Overview

This package contains the files required to simulate the ABB IRB 6640  manipulator (and variants) in Gazebo. 

Current version supports ROS Melodic


## Using Moveit! with Gazebo Simulator

1. Bring the robot model into gazebo and load the ros_control controllers:
2. Launch moveit! and ensure that it is configured to run alongside Gazebo:
```roslaunch abb_irb6640_gazebo moveit_planning_execution_gazebo.launch``` 
