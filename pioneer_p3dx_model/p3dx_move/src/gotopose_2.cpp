#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


bool newdata;
move_base_msgs::MoveBaseGoal _goal;
move_base_msgs::MoveBaseGoal _base;
	
	void callback(geometry_msgs::Pose destination){
	_goal.target_pose.header.frame_id = "map";
	ROS_INFO("new message");

  	_goal.target_pose.pose.position.x =destination.position.x;
  	_goal.target_pose.pose.position.y = destination.position.y;
  	_goal.target_pose.pose.orientation.w = destination.orientation.w;
	newdata=true;
	}
	


int main(int argc, char** argv){
	ROS_INFO("starting node");
   	ros::init(argc, argv, "pioneer_p3dx_2");
	ros::NodeHandle _nh;
	ros::Subscriber _topic_sub;
	ros::Publisher _topic_pub;
	
	MoveBaseClient _ac("p3dx_2/move_base", true);
	
	_base.target_pose.pose.position.x = 2.0;
	_base.target_pose.pose.position.y = -3.0;
	_base.target_pose.pose.orientation.w = 6.0;
	_base.target_pose.header.frame_id = "map";
	
	ROS_INFO("parameters inizialization done");
	while(!_ac.waitForServer()){
    ROS_INFO("Waiting for the move_base action server to come up");
  }	
  	_topic_sub = _nh.subscribe("/Pion/p3dx_2/destination", 3,callback);
	_topic_pub =_nh.advertise<std_msgs::Int8>("/Pion/p3dx_2/PosReached",3);
	
	ROS_INFO("topic started");
	newdata=false;
	ROS_INFO("newdata :%d",newdata);
	ros::Rate r(5);
	std_msgs::Int8 reached;
	ROS_INFO("starting the loop");
	while(ros::ok()){
		if(newdata){
		newdata=false;
		  ROS_INFO("Sending goal");
		  _goal.target_pose.header.stamp = ros::Time::now();
		  _ac.sendGoal(_goal);

		  _ac.waitForResult();
		  
		  if(_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		    ROS_INFO("Hooray, the base has almost finished");
		    
		    reached.data=1;
		    _topic_pub.publish(reached);
		    
		    _base.target_pose.header.stamp = ros::Time::now();
		    _ac.sendGoal(_base);
		    _ac.waitForResult();
		    if(_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		    		ROS_INFO("Hooray, the base is back");
				reached.data=2;
				_topic_pub.publish(reached);
		  
		    	}
		    }
		  else{reached.data=3;
				_topic_pub.publish(reached); //retry
		  	}
		    }

	
	ros::spinOnce();
	r.sleep();
	}
  
  return 0;
}
