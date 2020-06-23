# Smart Warehouse 2 - Robotics Lab 2020 - Master's Degree Automation Engineering University of Naples Federico II  

A ROS/Gazebo Warehouse_2 project for Robotics Lab made by Eliana La Frazia and Andrea Cavaliere (andreacavaliere49@gmail.com and lafrazia.eliana@gmail.com). The pioneer folder is an adaption of the Pioneer model made by Rafael Berkvens and Mario Serna Hernández. 
More details in the report.


# To install:
```
$ cd <catkin_ws>/src
$ git clone https://github.com/elianalf/RL_PROJECT.git 
$ cd ..
$ catkin_make
```

# To use:
```
To make sure your workspace is properly overlayed by the setup script:
$ source <catkin_ws>/devel/setup.sh
to launch the system see Execution Example

```
# Necessary dependencies :

```
--move base and navigation-- 
$ move_base sudo apt-get install ros-melodic-move-base ros-melodic-move-base-msgs 
$ gmapping sudo apt-get install ros-melodic-gmapping
$ amcl $ sudo apt-get install ros-melodic-amcl

--OpenCV--
OpenCv can be compiled and installed after downloaded the desired version on this page. https://github.com/opencv/opencv/tree/4.3.0.

--Move-it!, ros-industrial and ABB stack--
$ sudo apt-get install ros-melodic-moveit ros-melodic-moveit-plugins ros-melodic-moveit-planners
$ sudo apt-get install ros-melodic-industrial-core
$ sudo apt-get install ros-melodic-abb

---probably already downloaded dependencies but needed--
gazebo
gazebo_ros
gazebo_ros_control
gazebo_ros_pkgs
joint_state_controller
position_controllers
joint_trajectory_controller
robot_state_publisher
ros_controllers
```

# Execution Example:
```
every step has to be done in all separate shells

1)Launching Gazebo and spawning robots
$ roslaunch smart_warehouse_2 warehouse2.launch

2)Launch Move-it! Core
$ roslaunch abb_irb6640_gazebo moveit_planning_execution_gazebo.launch 

3)Run object-tracker node
$ rosrun smartwarehouse_opencv obj_tracker

4)Launch pioneer move base and AMCL
$ roslaunch p3dx_move moving_map.launch

5)Run Pioneer robot node
$ rosrun p3dx_move pioneer_p3dx_1

6) Run Pioneer manager node
$ rosrun p3dx_move p3dx_manager
```


# Note:
```
It's possible to launch a world with 2 Pioneers 3DX, it's not necessary to accomplish the task but it would have been a shame to delete it.
To make it available go to /smart_warehouse_2/warehouse.launch and uncomment the last part od the code, then between steps 5) and 6)  
5.1) 
$ roslaunch p3dx_move moving_map2.launch
5.2) 
$ rosrun p3dx_move pioneer_p3dx_1
to work together ad their best the system needs more testing, check the report for issues.


to create a new map, first launch the gazebo world (step 1) then in two separate shells
$ roslaunch p3dx_move moving_no_map.launch
$ roslaunch p3dx_move rviz.launch

```
