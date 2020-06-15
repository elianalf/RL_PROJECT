#include "abb_pp_task.h"


using namespace std;
using namespace tf;

bool task_completed;
geometry_msgs::Pose start_pose;
geometry_msgs::Pose pick_pose;
geometry_msgs::Pose place_pose;

PICK_PLACE_TASK::PICK_PLACE_TASK(string name_) :
  a_s(_n, name_, boost::bind(&PICK_PLACE_TASK::executeCB, this, _1), false),
    action_name(name_) {
        a_s.registerPreemptCallback( boost::bind(&PICK_PLACE_TASK::preemptCB, this) );
 topic_pub = _n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10);
 
   Rot_matrix_.setRPY(0, 0, 3.14); //rotation matrix to transform in robot base frame
   start_pose.orientation.x = 0;
   start_pose.orientation.y = 0.977;
   start_pose.orientation.z = 0;
	start_pose.orientation.w = 0.212;
	start_pose.position.x = 1.91;
	start_pose.position.y = 0;
	start_pose.position.z = 2.05;
	
	pick_pose = start_pose;
	
	place_pose = start_pose;
	
	double pxo = -1.4;
   double pyo = -2.8;
   double pzo = 0.15;
	place_pose.position.x = (Rot_matrix_[0].x() * pxo)+(Rot_matrix_[0].y() * pyo)+(Rot_matrix_[0].z()* pzo) + 0.5;
   place_pose.position.y = (Rot_matrix_[1].x() * pxo)+(Rot_matrix_[1].y() * pyo)+(Rot_matrix_[1].z()* pzo) - 1.5;
   place_pose.position.z = (Rot_matrix_[2].x() * pxo)+(Rot_matrix_[2].y() * pyo)+(Rot_matrix_[2].z()* pzo) + 0.2;
	 
  a_s.start();
   
}




void PICK_PLACE_TASK::moveit_abb(double px, double py, double pz){
     // ros::AsyncSpinner spinner(1);
      //spinner.start();
      
  static const std::string PLANNING_GROUP = "manipulator";
  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  //namespace rvt = rviz_visual_tools;
  //moveit_visual_tools::MoveItVisualTools visual_tools("world");
  //visual_tools.deleteAllMarkers();
  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  //visual_tools.loadRemoteControl();
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can print the name of the reference frame for this robot.
 // ROS_INFO_NAMED("abb_moveit_info", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  //ROS_INFO_NAMED("abb_moveit_info", "Reference Frame: %s", move_group.getPoseReferenceFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  // ROS_INFO_NAMED("abb_moveit_info", "End effector link: %s", move_group.getEndEffectorLink().c_str());
   
  // We can get a list of all the groups in the robot:
   //ROS_INFO_NAMED("abb_moveit_info", "Available Planning Groups:");
  //copy() copy a full vector or a part of it into a string copy(begin, end, where it has to copy)
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  std::ostream_iterator<std::string>(std::cout, ", "));
	
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   move_group.setMaxVelocityScalingFactor(1);
   move_group.setPlanningTime(10);
   bool success = false;
 
   
   pick_pose.position.x = px;
   pick_pose.position.y = py;
   pick_pose.position.z = pz;    
   
   
   move_group.setPoseTarget(pick_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose %s", success ? "" : "FAILED"); 
   move_group.move();
	usleep(5000);
	system("rosrun gazebo_ros_link_attacher attach.py");
	usleep(50000);
	success = false;
   
	move_group.setPoseTarget(start_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose %s", success ? "" : "FAILED");    
   move_group.move();
   usleep(10000);
   success = false;
   
   
	move_group.setPoseTarget(place_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose %s", success ? "" : "FAILED");  
   move_group.move();
   usleep(10000);
   const char* com_line="rosrun gazebo_ros_link_attacher detach.py";
   system(com_line);
	usleep(50000);
	success = false;
	
	move_group.setPoseTarget(start_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose %s", success ? "" : "FAILED");    
   move_group.move();
   usleep(5000);
   
   place_pose.position.x -= 0.3;
   //place_pose.position.y += ;
   
   smart_warehouse_2::box_posResult result;
   result.x_reached=px;
   result.y_reached=py;
   result.z_reached=pz;
   
   /***************************************************
   METTERE UN VERIFICA SUL SETTAGGIO A SUCCEEDED**********
   ****************************************/
   a_s.setSucceeded(result);
   cout << "SUCCEEDED" << endl;
   task_completed=true;
}


void PICK_PLACE_TASK::executeCB( const smart_warehouse_2::box_posGoalConstPtr &goal ){
 /* gazebo_msgs::ModelState sposta;
   sposta.model_name = "box_red_clone";
   sposta.pose.position.x = 0;
   sposta.pose.position.y =0;
   sposta.pose.position.z =0; 
    sposta.pose.orientation.x = 0;
   sposta.pose.orientation.y =0;
   sposta.pose.orientation.z =0;
   sposta.pose.orientation.w =0;
   sposta.reference_frame = "world";
   topic_pub.publish(sposta);
   cout<<"pub"<<endl;*/
   
   
   double pxo = goal->x_box;
   double pyo = goal->y_box;
   double pzo = goal->z_box;
   
   double pxr=(Rot_matrix_[0].x() * pxo)+(Rot_matrix_[0].y() * pyo)+(Rot_matrix_[0].z()* pzo) + 0.5;
   double pyr=(Rot_matrix_[1].x() * pxo)+(Rot_matrix_[1].y() * pyo)+(Rot_matrix_[1].z()* pzo) - 1.5;
   double pzr=(Rot_matrix_[2].x() * pxo)+(Rot_matrix_[2].y() * pyo)+(Rot_matrix_[2].z()* pzo) + 0.2;  
   
 /* double pxr = 1.91;
  double pyr = 0;
  double pzr = 0.8;*/
  
   cout<<"New pos in robot base frame: "<<pxr<<" "<<pyr<<" "<<pzr<<endl;
   while (!a_s.isPreemptRequested() && !task_completed) {
      cout<<"moveit abb"<<endl;
      moveit_abb( pxr , pyr,  pzr);
      
   }
   task_completed=false;
}

void PICK_PLACE_TASK::preemptCB(){
   cout<<"Got preempted"<<endl;
   //as.setPreempted(result,"I got Preempted");}

}

void PICK_PLACE_TASK::run(){
   ros::spin();
   
}

int main(int argc, char** argv){
  ros::init(argc, argv, "abb6640_moveit_node");
  PICK_PLACE_TASK pp("abb_robot_server");
  pp.run();
   
  return 0;

}
