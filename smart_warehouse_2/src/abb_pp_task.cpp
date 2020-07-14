#include "abb_pp_task.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


using namespace std;
using namespace tf;


geometry_msgs::Pose start_pose;
geometry_msgs::Pose pick_pose;
geometry_msgs::Pose place_pose;
smart_warehouse_2::box_posGoal _GOAL;


double pxo_red =1.95;
double pyo_red = 1.8;
double pzo_ = 0.31;
double pxo_blue = 1.95;
double pyo_blue = 2.15;
//position of the robot
float abb_z;
float abb_y;
float abb_x;
float abb_yaw;
//needed for cartesianPath
const double jump_threshold = 0.0;
const double eef_step = 0.02;
double fraction;
std::vector<geometry_msgs::Pose> waypoints_to_start;

Eigen::Matrix<double,3,3> rotation_matrix3;


//LOAD COORDINATE COMPONENT
void load_param( float & p, float def, string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

PICK_PLACE_TASK::PICK_PLACE_TASK(string name_) :
  a_s(_n, name_, boost::bind(&PICK_PLACE_TASK::executeCB, this, _1), false),
    action_name(name_) {
        a_s.registerPreemptCallback( boost::bind(&PICK_PLACE_TASK::preemptCB, this) );
        
   	Pioneer_pub=_n.advertise<std_msgs::String>("/Warehouse/NewBox",20);
 
        Rot_matrix_.setRPY(0, 0, 3.14); //** Rotation matrix to transform in robot base frame **
	float def=0.0;
        load_param( abb_yaw, def , "abb_yaw" );
        load_param( abb_x, def , "abb_x" );
        load_param( abb_y,def , "abb_y" );
        load_param( abb_z, def , "abb_z" );
        
        Eigen::Vector3f ea(abb_yaw, 0 ,0);
    
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
	
	start_pose.orientation.x = 0;
        start_pose.orientation.y = 1;
        start_pose.orientation.z = 0;
	start_pose.orientation.w = 0;
	start_pose.position.x = 1.91;
	start_pose.position.y = 0;
	start_pose.position.z = 1.9;
	
	pick_pose = start_pose;
   
	place_pose = start_pose;
	
	place_pose.position.x = pxo_red;
        place_pose.position.y = pyo_red;
        place_pose.position.z = pzo_;
	
     
  a_s.start();
  
   
}

void PICK_PLACE_TASK::abort_handler(){
   //** Try to reach the start position **
   bool start_success = false;
   cout<<"ABORT: CHECK ON RVIZ. MAYBE A REBOOT IS NEEDED!"<<endl;
   static const std::string PLANNING_GROUP = "manipulator";
   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
   const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   move_group.setMaxVelocityScalingFactor(1);
   move_group.setPlannerId ("RRTstarkConfigDefault");
   move_group.setPlanningTime(4);
   usleep(1000000);
   move_group.clearPoseTargets();
   move_group.setStartStateToCurrentState();
   usleep(500000);
   move_group.setPoseTarget(start_pose);
   start_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Start pose: %s", start_success ? "REACHABLE POSITION" : "FAILED");    
   start_success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   usleep(2000000);
  
}


bool PICK_PLACE_TASK::moveit_abb(double px, double py, double pz){
	bool task_completed;
	smart_warehouse_2::box_posResult result;   
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
	std::ostream_iterator<std::string>(std::cout, ", "));

	std_msgs::String to_p3dx;
	to_p3dx.data=_GOAL.box_name;

	move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();

	place_pose.position.z = 1;
	if(_GOAL.color == "red"){  
	   cout<<"it's red"<<endl;
	   pxo_red -= 0.3;
	   if(pxo_red < -0.7) // ** If it's the end of the station, start from the head **
	      pxo_red = 1.7;
	place_pose.position.x = pxo_red;
	place_pose.position.y = pyo_red;
	//** For parsing of Pioneer_manager **
	to_p3dx.data+="**red";
	}
	else if (_GOAL.color == "blue"){
	cout<<"it's blue"<<endl;
	pxo_blue -= 0.3;
	if(pxo_blue < -0.7) 
	 pxo_blue = 1.7;
	place_pose.position.x = pxo_blue;
	place_pose.position.y = pyo_blue; 
	//** For parsing of Pioneer_manager **
	to_p3dx.data+="*blue";
	}
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	move_group.setMaxVelocityScalingFactor(1);
	move_group.setPlannerId ("RRTstarkConfigDefault");
	move_group.setPlanningTime(15);
	std::vector<geometry_msgs::Pose> waypoints_to_pick;
	std::vector<geometry_msgs::Pose> waypoints_to_place;
	moveit_msgs::RobotTrajectory trajectory;
	
	bool success = false;
	bool success_pick = false;
	bool success_place=false;

	pick_pose.position.x = px;
	pick_pose.position.y = py;
	pick_pose.position.z = 1.1;    


	//** Attach command for the shell **
	string shell_command_attach;
	shell_command_attach+="rosservice call /link_attacher_node/attach '{model_name_1: 'abb_irb6640_185_280', link_name_1: 'link_6', model_name_2: '";
	shell_command_attach+=_GOAL.box_name;
	shell_command_attach+="', link_name_2: 'link'}'";
	//** Detach command for the shell ** 
	string shell_command_detach;
	shell_command_detach+="rosservice call /link_attacher_node/detach '{model_name_1: 'abb_irb6640_185_280', link_name_1: 'link_6', model_name_2: '";
	shell_command_detach+=_GOAL.box_name;
	shell_command_detach+="', link_name_2: 'link'}'";
	
	//** PICK BOX **
	//move_group.setPoseTarget(pick_pose);
	//success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	//ROS_INFO_NAMED("abb_moveit_info", "High pick pose: %s", success ? "REACHABLE POSITION" : "FAILED"); 
	//success_pick = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	
	move_group.setStartStateToCurrentState();
	waypoints_to_pick.push_back(pick_pose); //approaching pose
	pick_pose.position.z =pz;
	waypoints_to_pick.push_back(pick_pose); //real pick pose
	usleep(50);

//	move_group.clearPoseTargets();
//	move_group.setStartStateToCurrentState();
//	pick_pose.position.z =pz;


/*	move_group.setPoseTarget(pick_pose);
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("abb_moveit_info", "Pick pose: %s", success ? "REACHABLE POSITION" : "FAILED"); 
	success_pick = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	*/
	fraction = move_group.computeCartesianPath(waypoints_to_pick, eef_step, jump_threshold, trajectory);
	 ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
	 success_pick= (move_group.execute(trajectory)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	usleep(500);
	if(fraction==1.0){ // ** Attach **
	     ROS_INFO_STREAM("attaching...");
	     system(shell_command_attach.c_str());
	}
	else{
	   cout<<"Pick box failed!"<<endl;
	   abort_handler();
	   result.x_reached=1000.0;
		result.y_reached=1000.0;
		result.z_reached=1000.0;
		a_s.setAborted (result,"Pick box failed!" );
		task_completed=true;
		//** Return in order to wait for receiving the same position and retry **
		return task_completed;
	}
	usleep(10000);
	success = false;

	move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
	usleep(50);

	pick_pose.position.z = 1.1;
	waypoints_to_place.push_back(pick_pose);
	waypoints_to_place.push_back(place_pose);
	place_pose.position.z = pzo_;

	waypoints_to_place.push_back(place_pose);

	fraction = move_group.computeCartesianPath(waypoints_to_place, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


	//** PLACE BOX **
	success_place= (move_group.execute(trajectory)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("abb_moveit_info", "execution: %s", success ? "SUCCESS" : "FAILED");  
	if(!(fraction==1.0)){
	cout<<"Place pose failed"<<endl;
	abort_handler();
	   usleep(500000);
	   move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
	usleep(500000);
	//** Now try to reach the place pose from the start pose
	   move_group.setPoseTarget(place_pose);
	success = (move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("abb_moveit_info", "High place pose: %s", success ? "REACHABLE POSITION" : "FAILED");  
	success_place = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
	place_pose.position.z = pzo_;


	move_group.setPoseTarget(place_pose);
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	//ROS_INFO_NAMED("abb_moveit_info", "Place pose: %s", success ? "REACHABLE POSITION" : "FAILED");  
	success_place = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	usleep(50);
	}
	if(fraction==1.0){ //** Detach **
	system(shell_command_detach.c_str());
	ROS_INFO_STREAM("detaching... and publishing");
	Pioneer_pub.publish(to_p3dx);
	}

	usleep(10000);
	success = false;
	
	
	move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
	fraction = move_group.computeCartesianPath(waypoints_to_start, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
	
	success = (move_group.execute(trajectory)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("abb_moveit_info", "execution to start pose: %s", success ? "SUCCESS" : "FAILED");  
	
	usleep(50);


	//move_group.setPoseTarget(start_pose);
	//success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	//ROS_INFO_NAMED("abb_moveit_info", "Start pose: %s", success ? "REACHABLE POSITION" : "FAILED");    
	//move_group.move();
	usleep(50);

	move_group.clearPoseTargets();
	usleep(50);

	result.x_reached=px;
	result.y_reached=py;
	result.z_reached=pz;

	if(success_place&&success_pick)
	   { a_s.setSucceeded(result);
	     cout << "SUCCEEDED" << endl;}
	else{cout<<"THE SYSTEM HAS ABORTED. Maybe obj_tracker and RVIZ should be stopped and rerun. Exit."<<endl;
	  exit(0);
	}

	task_completed=true;
	return task_completed;
	}


void PICK_PLACE_TASK::executeCB( const smart_warehouse_2::box_posGoalConstPtr &goal ){

   _GOAL=*goal;
   bool task_complete=false;
   double pxo = goal->x_box;
   double pyo = goal->y_box;
   double pzo = goal->z_box;
   /* OLD 
   double pxr=(Rot_matrix_[0].x() * pxo)+(Rot_matrix_[0].y() * pyo)+(Rot_matrix_[0].z()* pzo)+0.5;
   double pyr=(Rot_matrix_[1].x() * pxo)+(Rot_matrix_[1].y() * pyo)+(Rot_matrix_[1].z()* pzo)-1.5;
   double pzr=(Rot_matrix_[2].x() * pxo)+(Rot_matrix_[2].y() * pyo)+(Rot_matrix_[2].z()* pzo) + 0.18;   
   
   cout<<"New pos. in robot base frame: "<<pxr<<" "<<pyr<<" "<<pzr<<endl; */
   
   Eigen::Matrix<double,3,1> traslation(abb_x,abb_y,abb_z+0.18);
   Eigen::Matrix<double,3,1> pose(pxo,pyo,pzo);
   pose=rotation_matrix3*pose+traslation;
   double pxr=pose[0];
   double pyr=pose[1];
   double pzr=pose[2];
   cout<<"New pos. in robot base frame with EIGEN : "<<pose<<endl;
  // while (!a_s.isPreemptRequested() && !task_completed) {
  
  while ( !task_complete) {
      task_complete=moveit_abb( pxr , pyr,  pzr);
   }
   task_complete=false;
}

void PICK_PLACE_TASK::preemptCB(){
   cout<<"Got preempted"<<endl;

}

void PICK_PLACE_TASK::run(){
   cout<<"moveit abb"<<endl;
   waypoints_to_start.push_back(start_pose);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "abb6640_moveit_node");
  ros::AsyncSpinner spinner(1);
  ros::Rate r(1);
  if(spinner.canStart()){ ROS_INFO(" start spinner");spinner.start();}
  PICK_PLACE_TASK pp("abb_robot_server");
  pp.run();
   while(ros::ok()) {r.sleep();}
   spinner.stop();
   
  return 0;
}
