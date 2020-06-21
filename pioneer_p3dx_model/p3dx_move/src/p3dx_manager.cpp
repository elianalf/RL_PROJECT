#include <ros/ros.h>
#include "boost/thread.hpp"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelState.h>

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
std::string box_name;
gazebo_msgs::ModelState sposta1;
gazebo_msgs::ModelState sposta2;
geometry_msgs::Pose blue_station;
geometry_msgs::Pose red_station;
geometry_msgs::Pose goal_p1;
geometry_msgs::Pose goal_p2;
double redstation_y ;
double bluestation_y ;
int red_index;
int blue_index;

class Pioneer_manager{
	public:
		Pioneer_manager();
		void run();
		void virtual_joint(std::string name,int p3, int att_or_det);
		
		
	private:
		ros::NodeHandle _nh;
		ros::Subscriber p3dx_1_sub;
		ros::Subscriber p3dx_2_sub;
		ros::Subscriber new_box;
		ros::Publisher gazebo_pub;
		ros::Publisher p3dx_1_pub;
		ros::Publisher p3dx_2_pub;
		bool p3dx_1_ready;
		bool p3dx_2_ready;
		void managing();
		std::vector<std::string> red_names;
		std::vector<std::string> blue_names;
		std::vector<int> _boxes; //0=blue , 255=rosso
		void incomingBox(const std_msgs::String );
		void p3dx_1_list(std_msgs::Int8 result);
		void p3dx_2_list(std_msgs::Int8 result);
	
};

Pioneer_manager::Pioneer_manager(){
	p3dx_1_sub = _nh.subscribe("/Pion/p3dx_1/PosReached", 10, &Pioneer_manager::p3dx_1_list, this);
	p3dx_1_pub =_nh.advertise<geometry_msgs::Pose>("/Pion/p3dx_1/destination",10);
	
	p3dx_2_sub = _nh.subscribe("/Pion/p3dx_2/PosReached", 10,&Pioneer_manager::p3dx_2_list, this);
	p3dx_2_pub =_nh.advertise<geometry_msgs::Pose>("/Pion/p3dx_2/destination",10);
	
	new_box = _nh.subscribe("/Warehouse/NewBox", 10,&Pioneer_manager::incomingBox,this);
	
	gazebo_pub =_nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1); 
	sposta1.pose.orientation.x = 0;
	sposta1.pose.orientation.y =0;
	sposta1.pose.orientation.z =0;
	sposta1.pose.orientation.w =1;
	redstation_y = 8.232;
	bluestation_y = -10.832;
	p3dx_1_ready=true;
	p3dx_2_ready=false;
	
	
}



void Pioneer_manager::virtual_joint(std::string name,int p3, int att_or_det){

std::string shell_command;
   
if(att_or_det==1){
	if(p3==2){
	   shell_command="rosservice call /link_attacher_node/attach '{model_name_1: 'p3dx_2', link_name_1: 'base_link', model_name_2: '";}
	else{
	   shell_command="rosservice call /link_attacher_node/attach '{model_name_1: 'p3dx_1', link_name_1: 'base_link', model_name_2: '";}

	   shell_command+=name;
	   shell_command+="', link_name_2: 'link'}'";
	}
	
else if(att_or_det==2){

	if(p3==2){
	   shell_command="rosservice call /link_attacher_node/detach '{model_name_1: 'p3dx_2', link_name_1: 'base_link', model_name_2: '";
	   	}
	   	
	else{
	   shell_command="rosservice call /link_attacher_node/detach '{model_name_1: 'p3dx_1', link_name_1: 'base_link', model_name_2: '";}
	   shell_command+= name;
	   shell_command+="', link_name_2: 'link'}'";
	}

system(shell_command.c_str());


}



void Pioneer_manager::incomingBox(const std_msgs::String box_type ){


ROS_INFO("new box request, color is:");
box_name=box_type.data;
std::size_t found=box_type.data.find("**red");
box_name.erase(box_name.end()-5,box_name.end());
if(found!=box_type.data.npos){
	ROS_INFO("red"); 
	_boxes.push_back(255);
	red_names.push_back(box_name);
	ROS_INFO_STREAM(box_name);
	}
else{  	ROS_INFO("blue"); 
	_boxes.push_back(0);
	blue_names.push_back(box_name);
	ROS_INFO_STREAM(box_name);
	}

}

void Pioneer_manager::managing(){
	ros::Rate r(2);
	int red_index=0;
	int blue_index=0;
	int index=0;
	bool still_a_box=true;
	while(ros::ok()){
		if((!_boxes.empty())){
			try{
				if(_boxes.at(index)==0) ROS_INFO("next one is a blue box"); //blue_box
				else ROS_INFO("next one is a red box");                    //red_box
				}
			catch(const std::out_of_range& out){
				ROS_INFO("we completed the queue,waiting for new ones.");
				//you already delivered all the boxes, erase the queue
				index=0;
				_boxes.erase(_boxes.begin(),_boxes.end());
				_boxes.resize(0);
				still_a_box=false;
			}
			if(still_a_box){
			while((!p3dx_1_ready)&&(!p3dx_2_ready)){r.sleep();}
			ROS_INFO("at least one pioneer is ready");
			if(_boxes[index]==0){
				if(p3dx_1_ready){ index++; 
					 ROS_INFO(" blue to p3dx_1.");
					 sposta1.model_name = blue_names[blue_index];
					 blue_index++;
					 sposta1.pose.position.x = 2;   
					 sposta1.pose.position.y =-4;
					 sposta1.pose.position.z =0.310;
					 sposta1.pose.orientation.x = 0;
					 sposta1.pose.orientation.y =0;
					 sposta1.pose.orientation.z =0;
					 sposta1.pose.orientation.w =1;
					 sposta1.reference_frame="world";
					 gazebo_pub.publish(sposta1);
					 usleep(1000000);
					 virtual_joint(box_name,1,1);
					 usleep(100000);
					// sleep(2);	 
					p3dx_1_pub.publish(blue_station);
					 sleep(2);
					goal_p1=blue_station;
					p3dx_1_ready=false;
							}
					else if(p3dx_2_ready){ index++;
					 sleep(2);
					 ROS_INFO(" blue to p3dx_2."); 
					 sposta2.model_name = blue_names[blue_index];
					 blue_index++;		 
					 p3dx_2_pub.publish(blue_station);
					  sleep(2);
					 goal_p2=blue_station;
					 sposta2.model_name = box_name;
					p3dx_2_ready=false;
					}
			}
			else if(_boxes[index]==255){
					if(p3dx_1_ready){ index++;
					
					 ROS_INFO(" red to p3dx_1.");
					 sposta1.model_name = red_names[red_index];
					 red_index++;
					 sposta1.pose.position.x = 2;   
					 sposta1.pose.position.y =-4;
					 sposta1.pose.position.z =0.330;
					 sposta1.pose.orientation.x = 0;
					 sposta1.pose.orientation.y =0;
					 sposta1.pose.orientation.z =0;
					 sposta1.pose.orientation.w =1;
					 sposta1.reference_frame="world";
					 
					 gazebo_pub.publish(sposta1);
					 usleep(1000000);
					 virtual_joint(box_name,1,1);	 	
					 usleep(100000);
					  //sleep(2);	 
					 p3dx_1_pub.publish(red_station);
					 sleep(2);
					 goal_p1=red_station;
					 p3dx_1_ready=false;
					 		}
					else if(p3dx_2_ready){ index++;
					 ROS_INFO(" red to p3dx_2.");
					 sposta2.model_name = red_names[red_index];
					 red_index++;
					  sleep(2); 		 
					 p3dx_2_pub.publish(red_station);
					 goal_p2=red_station;
					 sleep(2);	
					p3dx_2_ready=false;
								}
					}
			}
		}
		else{
		still_a_box=true;
		index=0;
		}
			
	
		r.sleep();
	}
	
	

}


void Pioneer_manager::p3dx_1_list(std_msgs::Int8 result){
 if(result.data==2){p3dx_1_ready=true; ROS_INFO("p3dx_1 is back.");}
 else if(result.data==1){
virtual_joint(sposta1.model_name,1,2);
 if(goal_p1.position.y=red_station.position.y){
 	sposta1.pose.position.x = -1.2902;   
	redstation_y+=0.3;
	sposta1.pose.position.z =0.310;
	sposta1.pose.position.y = redstation_y;
	sposta1.reference_frame="world";
	gazebo_pub.publish(sposta1);
 	}
 else{
 	sposta1.pose.position.x = -1.1502;   
	bluestation_y+=0.3;
	sposta1.pose.position.z =0.310;
	sposta1.pose.position.y = bluestation_y;
	sposta1.reference_frame="world";
 	gazebo_pub.publish(sposta1);
 }
 
 
  ROS_INFO("p3dx_1 is coming back.");}
 else{ //retry
 		p3dx_1_pub.publish(goal_p1);
 
 }
}

void Pioneer_manager::p3dx_2_list(std_msgs::Int8 result){
  if(result.data==2) {p3dx_2_ready=true;  ROS_INFO("p3dx_2 is back.");}
 else if(result.data==1){
 //virtual_joint(sposta2.model_name,2,2);
 if(goal_p2.position.y=red_station.position.y){
 	sposta2.pose.position.x = -1.2902;   
	sposta2.pose.position.y =8.232;
	sposta2.pose.position.z =0.2910;
	sposta2.pose.orientation.x = 0;
	sposta2.pose.orientation.y =0;
	sposta2.pose.orientation.z =0;
	sposta2.pose.orientation.w =1;
	sposta2.reference_frame="world";
	//gazebo_pub.publish(sposta2);
 	}
 else{
 	sposta2.pose.position.x = -1.2902;   
	sposta2.pose.position.y =-8.232;
	sposta2.pose.position.z =0.2910;
	sposta2.pose.orientation.x = 0;
	sposta2.pose.orientation.y =0;
	sposta2.pose.orientation.z =0;
	sposta2.pose.orientation.w =1;
	sposta2.reference_frame="world";
 	//gazebo_pub.publish(sposta2);
 }
 
 
 
  ROS_INFO("p3dx_1 is coming back.");}
 else{ //retry
 		p3dx_2_pub.publish(goal_p2);
 
 }
}

void Pioneer_manager::run(){

	boost::thread managing_t( &Pioneer_manager::managing, this);
	ros::spin();
}



int main(int argc, char** argv) {
	red_station.position.x = 2.0;
	red_station.position.y = 9.6;
	red_station.orientation.w = 6.0;

	blue_station.position.x = 2.0;
	blue_station.position.y = -9.6;
	blue_station.orientation.w = 6.0;
	red_index=0;
	blue_index=0;
		
	ros::init(argc, argv, "p3dx_manager");
	Pioneer_manager p3dx_m;
	p3dx_m.run();
	

return 0;
}

