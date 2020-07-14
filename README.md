# Smart Warehouse 2 - Robotics Lab 2020 - Master's Degree Automation Engineering University of Naples Federico II  

A ROS/Gazebo Warehouse_2 project for Robotics Lab made by Eliana La Frazia and Andrea Cavaliere (andreacavaliere49@gmail.com and lafrazia.eliana@gmail.com). The pioneer folder is an adaption of the Pioneer model made by Rafael Berkvens and Mario Serna Hernández. 
More details in the report.

# UPDATES
```
update introduces new features. 
-It is now available to launch the simulation just with two .launch file:
1) launch the world and  pioneers' nodes
$ roslaunch smart_warehouse_2 all.launch
using the parameter twopioneer(boolean parameter) you can choose to simulate the warehouse with 1 or 2 pioneers( obviously this will influence the box management policy)
2) launch the camera and manipulator node
$ roslaunch smart_warehouse_2 manipulator.launch

-The camera node now has a better and smarter policy to choose the box to pick (see the report for details).
-The manipulator movements are now smoother and faster.
-The robots adapt to every movement of the object in the world.
-any position change can be done in the .launch file


```

# To install:
```
$ cd <catkin_ws>/src
$ git clone https://github.com/elianalf/RL_PROJECT.git 
$ cd ..
$ catkin_make

If there are same actionlib errors during the compilation of the packages "smart_warehouse_2" and "smartwarehouse_opencv" it is recommended:
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="smart_warehouse_2"
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="smartwarehouse_opencv"
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
move_base 
$  sudo apt-get install ros-melodic-move-base ros-melodic-move-base-msgs 
gmapping 
$  sudo apt-get install ros-melodic-gmapping
amcl
$ sudo apt-get install ros-melodic-amcl

$ sudo apt-get install ros-melodic-map-server
$ sudo apt-get install ros-melodic-dwa-local-planner

--OpenCV--
OpenCv can be compiled and installed after downloaded the desired version on this page: https://github.com/opencv/opencv/tree/4.3.0

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
If the user needs to tune the rgb exact values the camera has to detect, he/she can follow this step.  The default colours are red and blue.
Every step has to be done in all separate shells:

1)Launching Gazebo and spawning robots
$ roslaunch smart_warehouse_2 warehouse2.launch

2)Go to /smartwarehouse_opencv/launch/identify_boxes.launch and change the value of the parameter "set_RGB" to TRUE and launch it.
$ roslaunch smartwarehouse_opencv identify_boxes.launch

3)Define the desired ranges of rgb values and change them into the identify_boxes.launch. Then change the value of the parameter "set_RGB" to FALSE and launch it again as the previous step 2). Finally execute the step 2) (Launch Move-it! Core) and from step 4) onwards of the following steps. 
```
```
If the user doesn't need to tune the rgb values, he/she can follow this step.
Every step has to be done in all separate shells:

1)Launching Gazebo and spawning robots
$ roslaunch smart_warehouse_2 warehouse2.launch

2)Launch Move-it! Core. After the launch it's possible to close the gui of RVIZ (RVIZ window).
$ roslaunch abb_irb6640_gazebo moveit_planning_execution_gazebo.launch 

3)Run object-tracker node 
$ rosrun smartwarehouse_opencv obj_tracker

4)Launch pioneer move base and AMCL
$ roslaunch p3dx_move moving_map.launch

5)Run Pioneer robot node
$ rosrun p3dx_move pioneer_p3dx_1

6) Run Pioneer manager node
$ rosrun p3dx_move p3dx_manager

7)Run ABB pick and place task manager
$ rosrun smart_warehouse_2 abb_pp_task 
```


# Note:
```
It's possible to launch a world with 2 Pioneers 3DX, it's not necessary to accomplish the task but it would have been a shame to delete it.
To make it available go to /smart_warehouse_2/warehouse.launch and uncomment the last part od the code, in  pioneer_p3dx_model/p3dx_move/src/p3dx_manager.cpp set the variable p3dx_2_ready to "true"(row 86), get back to catkin_ws/ and launch the command "$ catkin_make". Then between steps 5) and 6)
5.1) 
$ roslaunch p3dx_move moving_map2.launch
5.2) 
$ rosrun p3dx_move pioneer_p3dx_2
to work together ad their best the system needs more testing, check the report for issues.


to create a new map, first launch the gazebo world (step 1) then in two separate shells
$ roslaunch p3dx_move moving_no_map.launch
$ roslaunch p3dx_move rviz.launch




to make the "ray beam" disappear during the simulation go to pioneer_p3dx_model/p3dx_description/urdf/pioneer3dx.gazebo, row 97 and set "visualize" to false.
```
