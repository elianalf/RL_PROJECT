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
geometry_msgs::Pose p3dx_1_idle;
geometry_msgs::Pose p3dx_2_idle;

geometry_msgs::Pose goal_p1;
geometry_msgs::Pose goal_p2;
double redstation_y ;
double bluestation_y ;
double redstation_x ;
double bluestation_x ;
int red_index;
int blue_index;
int counter_red;
int counter_blue;



//TUNING TRUE OR FALSE
void load_param( bool & p, bool def, std::string name ) {
  ros::NodeHandle n_param("~");
  
std::cout<<"in true false";
  if( !n_param.getParam( name, p)){  p = def;}
  
std::cout<<" \n \n done true \n";
}

//LOAD COORDINATE COMPONENT
void load_param( double & p, double def, std::string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
  std::cout << name << ": " << "\t" << p << std::endl;
}


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
	redstation_x = -1.29;
	bluestation_x = -1.15;
	counter_red=0;
	counter_blue=0;
	p3dx_1_ready=true;
	bool buffer=true;
	load_param(p3dx_2_ready, buffer, "Twopioneer" );
	
	
	double buff_coordinate=0.0;
	load_param(buff_coordinate, 0.0, "p3dx_1_x" );
	p3dx_1_idle.position.x=buff_coordinate;
	load_param(buff_coordinate, 0.0, "p3dx_1_y" );
	p3dx_1_idle.position.y=buff_coordinate;
	load_param(buff_coordinate, 0.0, "p3dx_2_x" );
	p3dx_2_idle.position.x=buff_coordinate;
	load_param(buff_coordinate, 0.0, "p3dx_2_y" );
	p3dx_2_idle.position.y=buff_coordinate;
	
ROS_INFO("p3dx_2_ready is : %d",p3dx_2_ready);
	
ROS_INFO("funziona");
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
	bool still_a_red_box=true;
	bool still_a_blue_box=true;
	ROS_INFO("starting managing");
	while(ros::ok()){
		
		if((!red_names.empty())){
			try{	
				if(red_names.at(red_index).find("red")) std::cout<<"red box, its names is:"<<red_names[red_index];
				}
			catch(const std::out_of_range& out){
				ROS_INFO("we completed the  red queue");
				//you already delivered all the boxes, erase the queue
				red_index=0;
				red_names.erase(red_names.begin(),red_names.end());
				red_names.resize(0);
				still_a_red_box=false;
			}
		}
		else {still_a_red_box=false;}
		if((!blue_names.empty())){
			try{
				if(blue_names.at(blue_index).find("blue")) std::cout<<"blu box, its names is:"<<blue_names[blue_index];
				}
			catch(const std::out_of_range& out){
				ROS_INFO("we completed the  blue queue,waiting for new ones.");
				//you already delivered all the boxes, erase the queue
				blue_index=0;
				blue_names.erase(blue_names.begin(),blue_names.end());
				blue_names.resize(0);
				still_a_blue_box=false;
			}
		}
		else {still_a_blue_box=false;}
			if((still_a_blue_box)||(still_a_red_box)){
			
			ROS_INFO("waiting the robots");
			while((!p3dx_1_ready)&&(!p3dx_2_ready)){r.sleep();}
			if((p3dx_1_ready)&&(still_a_blue_box)){
				
				         ROS_INFO(" blue to p3dx_1.");
				         
					 sposta1.model_name = blue_names[blue_index];
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
					 virtual_joint(blue_names[blue_index],1,1);
					 blue_index++;
					 usleep(100000);
					// sleep(2);	 
					p3dx_1_pub.publish(blue_station);
					goal_p1=blue_station;
					p3dx_1_ready=false;
				
			}
			if((p3dx_2_ready)&&(still_a_red_box)){
			
					 ROS_INFO(" red to p3dx_2.");
					 sposta2.model_name = red_names[red_index];
					 sposta2.pose.position.x = 2;   
					 sposta2.pose.position.y =-3;
					 sposta2.pose.position.z =0.330;
					 sposta2.pose.orientation.x = 0;
					 sposta2.pose.orientation.y =0;
					 sposta2.pose.orientation.z =0;
					 sposta2.pose.orientation.w =1;
					 sposta2.reference_frame="world";
					 
					 gazebo_pub.publish(sposta2);
					 usleep(1000000);
					 virtual_joint(red_names[red_index],2,1);	 	
					 usleep(100000);
					 red_index++;
					  //sleep(2);		 
					 p3dx_2_pub.publish(red_station);
					 goal_p2=red_station;	
					p3dx_2_ready=false;
							
					}
			}
			else{
			still_a_red_box=true;
			still_a_blue_box=true;
			index=0;
			red_index=0;
			blue_index=0;
			}
			
	
		r.sleep();
	}
	
	

}


void Pioneer_manager::p3dx_1_list(std_msgs::Int8 result){
 if(result.data==2){p3dx_1_ready=true; ROS_INFO("p3dx_1 is back.");}
 else if(result.data==1){
	virtual_joint(sposta1.model_name,1,2);	 	
	usleep(100000);
 	counter_blue++;
 	if(counter_blue==7){bluestation_y=-10.832; bluestation_x+=0.3;}
 	sposta1.pose.position.x = bluestation_x;   
	bluestation_y+=0.3;
	sposta1.pose.position.z =0.310;
	sposta1.pose.position.y = bluestation_y;
	sposta1.reference_frame="world";
 	gazebo_pub.publish(sposta1);
        ROS_INFO("p3dx_1 is coming back.");
        goal_p1=p3dx_1_idle;
        usleep(100000);
  }
 else{ //retry
 		p3dx_1_pub.publish(goal_p1);
 
 }
}

void Pioneer_manager::p3dx_2_list(std_msgs::Int8 result){
  if(result.data==2) {p3dx_2_ready=true;  ROS_INFO("p3dx_2 is back.");}
 else if(result.data==1){
 	virtual_joint(sposta2.model_name,2,2);
 	usleep(100000);
        counter_red++;
 	if(counter_red==7){redstation_y=8.232; redstation_x+=0.3;}
 	sposta1.pose.position.x = redstation_x;
	redstation_y+=0.3;
	sposta2.pose.position.z =0.310;
	sposta2.pose.position.y = redstation_y;
	sposta2.reference_frame="world";
	
	gazebo_pub.publish(sposta2);
	usleep(100000);
 	ROS_INFO("p3dx_2 is coming back.");
 	goal_p2=p3dx_2_idle;
 	}
 else{ //retry
 		p3dx_2_pub.publish(goal_p2);
 
 }
}

void Pioneer_manager::run(){

	ROS_INFO("run");
	boost::thread managing_t( &Pioneer_manager::managing, this);
	ros::spin();
}



int main(int argc, char** argv) {

	
	ROS_INFO("where all begins");
	ros::init(argc, argv, "p3dx_manager");
	red_station.position.x = 2.0;
	red_station.position.y = 9.6;
	red_station.orientation.w = 6.0;

	blue_station.position.x = 2.0;
	blue_station.position.y = -9.6;
	blue_station.orientation.w = 6.0;
	
	
	ROS_INFO("here in main");
	
	
	p3dx_1_idle.orientation.w = 6.0;
	p3dx_2_idle.orientation.w = 6.0;
	
	red_index=0;
	blue_index=0;
		
	Pioneer_manager p3dx_m;
	
	ROS_INFO("here in main");
	p3dx_m.run();
	
	

return 0;
}

