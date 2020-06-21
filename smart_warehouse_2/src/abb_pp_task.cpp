#include "abb_pp_task.h"


using namespace std;
using namespace tf;

bool task_completed;
geometry_msgs::Pose start_pose;
geometry_msgs::Pose pick_pose;
geometry_msgs::Pose place_pose;
smart_warehouse_2::box_posGoal _GOAL;

/*********************************+
   I VALORI SUCCESSIVI SONO LE POSIZIONI (IN ROBOT FRAME) INIZIALI IN CUI IL ROBOT DEVE INIZIARE A POSIZIONARE I BOX ROSSI E BLUE, POI IN MOVEIT_ABB C'È IL CAMBIO DELLA POSIZIONE A OGNI CHIAMATA DI FUNZIONE. BISOGNA TESTARE QUESTE POSIZIONI (E ANCHE LO SCORRIMENTO +=0.3)
*********************************/
double pxo_red =2;// -0.7;
double pyo_red = 1.8;
double pzo_ = 0.31;
double pxo_blue = 1.95;
double pyo_blue = 2.15;


PICK_PLACE_TASK::PICK_PLACE_TASK(string name_) :
  a_s(_n, name_, boost::bind(&PICK_PLACE_TASK::executeCB, this, _1), false),
    action_name(name_) {
        a_s.registerPreemptCallback( boost::bind(&PICK_PLACE_TASK::preemptCB, this) );
   //topic_pub = _n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10);
   	Pioneer_pub=_n.advertise<std_msgs::String>("/Warehouse/NewBox",20);
 
   Rot_matrix_.setRPY(0, 0, 3.14); //rotation matrix to transform in robot base frame
   /*start_pose.orientation.x = 0;
   start_pose.orientation.y = 0.977;
   start_pose.orientation.z = 0;
	start_pose.orientation.w = 0.212;*/
	start_pose.orientation.x = 0;
   start_pose.orientation.y = 1;
   start_pose.orientation.z = 0;
	start_pose.orientation.w = 0;
	start_pose.position.x = 1.91;
	start_pose.position.y = 0;
	start_pose.position.z = 2;
	
	pick_pose = start_pose;
	/*pick_pose.orientation.x = 0.7;
   pick_pose.orientation.y = 0.7;
   pick_pose.orientation.z = 0;
   pick_pose.orientation.w = 0;*/
   
	place_pose = start_pose;
	/*place_pose.orientation.x = 0.7;
   place_pose.orientation.y = -0.7;
   place_pose.orientation.z = 0;
   place_pose.orientation.w = 0;*/
	
	place_pose.position.x = pxo_red;
   place_pose.position.y = pyo_red;
   place_pose.position.z = pzo_;
	
  
  a_s.start();
  
   
}




void PICK_PLACE_TASK::moveit_abb(double px, double py, double pz){

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
      place_pose.position.x = pxo_red;
      place_pose.position.y = pyo_red;
      //per parsing del Pioneer_manager
      to_p3dx.data+="**red";
      }
   else if (_GOAL.color == "blue"){
   cout<<"	BLUE..."<<endl;
      pxo_blue -= 0.3;
      place_pose.position.x = pxo_blue;
      place_pose.position.y = pyo_blue; 
      //per parsing del Pioneer_manager
      to_p3dx.data+="*blue";
      }
	
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   //move_group.setEndEffectorLink ("link_6");
   move_group.setMaxVelocityScalingFactor(1);
   move_group.setPlannerId ("RRTstarkConfigDefault");
   move_group.setPlanningTime(4);
   bool success = false;
	bool success_pick = false;
	bool success_place=false;
   
   pick_pose.position.x = px;
   pick_pose.position.y = py;
   pick_pose.position.z = 1.1;    
   
   
   //costruzione del comando di attach da dare in shell
   string shell_command_attach;
   shell_command_attach+="rosservice call /link_attacher_node/attach '{model_name_1: 'abb_irb6640_185_280', link_name_1: 'link_6', model_name_2: '";
   shell_command_attach+=_GOAL.box_name;
   shell_command_attach+="', link_name_2: 'link'}'";
   //costruzione del comando di detach da dare in shell
   string shell_command_detach;
   shell_command_detach+="rosservice call /link_attacher_node/detach '{model_name_1: 'abb_irb6640_185_280', link_name_1: 'link_6', model_name_2: '";
   shell_command_detach+=_GOAL.box_name;
   shell_command_detach+="', link_name_2: 'link'}'";
   
   move_group.setPoseTarget(pick_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose 1: %s", success ? "REACHABLE POSITION" : "FAILED"); 
   success_pick = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   //move_group.move();
   usleep(500);
   move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
   pick_pose.position.z =pz;
   
   move_group.setPoseTarget(pick_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose 1: %s", success ? "REACHABLE POSITION" : "FAILED"); 
   success_pick = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   //move_group.move();
   usleep(500);
	if(success_pick){ //ROS_INFO_STREAM(shell_command_attach);
	     ROS_INFO_STREAM("attaching...");
	     system(shell_command_attach.c_str());
	}
	else{
		result.x_reached=-100.0;
		result.y_reached=-100.0;
		result.z_reached=-100.0;
		a_s.setAborted (result,"Pick box failed!" );
		task_completed=true;
	}
	usleep(10000);
	success = false;
	
   move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
   usleep(500);
   pick_pose.position.z = 1.1;
   move_group.setPoseTarget(pick_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose 1: %s", success ? "REACHABLE POSITION" : "FAILED"); 
   success_pick = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   //move_group.move();
   usleep(500);
   move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
	/*move_group.setPoseTarget(start_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose 2: %s", success ? "REACHABLE POSITION" : "FAILED");    
   move_group.move();
   usleep(500);
   
   success = false;
   
   move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
   usleep(500);*/
   
   move_group.setPoseTarget(place_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose 3: %s", success ? "REACHABLE POSITION" : "FAILED");  
   //move_group.move();
   success_place = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   usleep(500);
   move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
   place_pose.position.z = pzo_;
   
	move_group.setPoseTarget(place_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose 3: %s", success ? "REACHABLE POSITION" : "FAILED");  
   //move_group.move();
   success_place = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   usleep(500);
   system(shell_command_detach.c_str());
   if(success_place){
   		
      ROS_INFO_STREAM("detaching... and publishing");
   		//ROS_INFO_STREAM(shell_command_detach);
   		Pioneer_pub.publish(to_p3dx);
   }
   else{
		result.x_reached=1000.0;
		result.y_reached=1000.0;
		result.z_reached=1000.0;
		a_s.setAborted (result,"Place box failed!" );
		task_completed=true;
	}
	usleep(10000);
	success = false;
	
	move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
   usleep(500);
   
	move_group.setPoseTarget(start_pose);
   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   ROS_INFO_NAMED("abb_moveit_info", "Visualizing target pose 4: %s", success ? "REACHABLE POSITION" : "FAILED");    
   move_group.move();
   usleep(500);
   
   move_group.clearPoseTargets();
	move_group.setStartStateToCurrentState();
   usleep(500);
   
   result.x_reached=px;
   result.y_reached=py;
   result.z_reached=pz;
   
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
   _GOAL=*goal;
   
   double pxo = goal->x_box;
   double pyo = goal->y_box;
   double pzo = goal->z_box;
   
   double pxr=(Rot_matrix_[0].x() * pxo)+(Rot_matrix_[0].y() * pyo)+(Rot_matrix_[0].z()* pzo) + 0.5;
   double pyr=(Rot_matrix_[1].x() * pxo)+(Rot_matrix_[1].y() * pyo)+(Rot_matrix_[1].z()* pzo) - 1.5;
   double pzr=(Rot_matrix_[2].x() * pxo)+(Rot_matrix_[2].y() * pyo)+(Rot_matrix_[2].z()* pzo) + 0.18;  
   //double pzr=(Rot_matrix_[2].x() * pxo)+(Rot_matrix_[2].y() * pyo)+(Rot_matrix_[2].z()* pzo) + 0.2;  
   

   cout<<"New pos in robot base frame: "<<pxr<<" "<<pyr<<" "<<pzr<<endl;
   while (!a_s.isPreemptRequested() && !task_completed) {
      
      moveit_abb( pxr , pyr,  pzr);
      
   }
   task_completed=false;
}

void PICK_PLACE_TASK::preemptCB(){
   cout<<"Got preempted"<<endl;
   //as.setPreempted(result,"I got Preempted");}

}

void PICK_PLACE_TASK::run(){
   
 
     cout<<"moveit abb"<<endl;
	
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
