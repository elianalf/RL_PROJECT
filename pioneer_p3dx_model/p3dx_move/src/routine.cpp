#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//base pose for p3dx_1
move_base_msgs::MoveBaseGoal base_1;



//base pose for p3dx_2
move_base_msgs::MoveBaseGoal base_2;



//red box station
move_base_msgs::MoveBaseGoal red_station;




//blu box station
move_base_msgs::MoveBaseGoal blue_station;





int main(int argc, char** argv){
base_1.target_pose.pose.position.x = 2.0;
base_1.target_pose.pose.position.x = -4.0;
base_1.target_pose.pose.orientation.w = 6.0;
base_1.target_pose.header.frame_id = "map";

base_2.target_pose.pose.position.x = 2.0;
base_2.target_pose.pose.position.x = -3.0;
base_2.target_pose.pose.orientation.w = 6.0;
base_2.target_pose.header.frame_id = "map";

red_station.target_pose.pose.position.x = 0.0;
red_station.target_pose.pose.position.x = 9.6;
red_station.target_pose.pose.orientation.w = 6.0;
red_station.target_pose.header.frame_id = "map";

blue_station.target_pose.pose.position.x = 0.0;
blue_station.target_pose.pose.position.x = -9.6;
blue_station.target_pose.pose.orientation.w = 6.0;
blue_station.target_pose.header.frame_id = "map";
ros::init(argc, argv, "p3dx_navigation");
 std::string value_from_cl = argv[1];
  if (argc > 5)
       ROS_INFO("hope you choose a good name");
  else{
  	ROS_INFO("not a valid name, try with p3xd_1 or p3xd_2 otherwise read properly the READ.ME file to understand how to insert a custom name");
  	
    return 0;}
  std::string mb_serv=value_from_cl+"/move_base";
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  if(value_from_cl.c_str()=="p3dx_1") 

     MoveBaseClient ac(mb_serv, true);
  if((!value_from_cl.c_str()=="p3dx_2")&&(!value_from_cl.c_str()=="p3dx_1")){ 
    ROS_INFO("not a valid name, try with p3xd_1 or p3xd_2 otherwise read properly the READ.ME file to understand how to insert a custom name");
    return 0;}
  //wait for the action server to come up

 while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
return 0;
}
