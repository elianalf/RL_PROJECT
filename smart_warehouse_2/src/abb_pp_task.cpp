
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb6640_moveit_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  

   static const std::string PLANNING_GROUP = "manipulator";
  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("abb_moveit_info", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("abb_moveit_info", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("abb_moveit_info", "Available Planning Groups:");
    //copy() copy a full vector or a part of it into a string copy(begin, end, where it has to copy)
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  std::ostream_iterator<std::string>(std::cout, ", "));
	
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   bool success = false;
 
	geometry_msgs::Pose start_pose;
   start_pose.orientation.x = 0;
   start_pose.orientation.y = 0.977;
   start_pose.orientation.w = 0;
	start_pose.orientation.w = 0.212;
	start_pose.position.x = 1.91;
	start_pose.position.y = 0;
	start_pose.position.z = 2.05;
	
   geometry_msgs::Pose pick_pose = start_pose;
  pick_pose.position.x = 1.76;
  pick_pose.position.y = -0.97;
  pick_pose.position.z = 0.43;    
  move_group.setMaxVelocityScalingFactor(1.1);
   move_group.setPoseTarget(pick_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose %s", success ? "" : "FAILED");  visual_tools.trigger();  
   move_group.move();
   
	usleep(5);
	
	 move_group.setPoseTarget(start_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose %s", success ? "" : "FAILED");  visual_tools.trigger();  
   move_group.move();
   
  return 0;

}
