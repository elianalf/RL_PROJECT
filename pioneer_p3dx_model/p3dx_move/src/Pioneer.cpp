#include "Pioneer.h"

#include "boost/thread.hpp"

Pioneer::Pioneer(MoveBaseClient* ac){
_ac=ac;
_base.target_pose.pose.position.x = 2.0;
_base.target_pose.pose.position.x = -4.0;
_base.target_pose.pose.orientation.w = 6.0;
_base.target_pose.header.frame_id = "map";

//_new_box = _nh.subscribe(/red_box, 0, &Pioneer::newBoxCb(), this );
_delivered=true;
_ready=true;

            donuts_=false;
}


Pioneer::Pioneer(MoveBaseClient* ac_spec,const move_base_msgs::MoveBaseGoal start_pose){
_ac=ac_spec;
_base.target_pose.pose.position.x = start_pose.target_pose.pose.position.x;
_base.target_pose.pose.position.y = start_pose.target_pose.pose.position.y ;
_base.target_pose.pose.orientation.w = start_pose.target_pose.pose.orientation.w;
_base.target_pose.header.frame_id = "map";
_delivered=true; //così se si esegue il "run" prima di "goto" non parte a caso
_ready=true;

            donuts_=false;
}

void Pioneer::go_to ( const move_base_msgs::MoveBaseGoal goal_pose){

_goal.target_pose.pose.position.x = goal_pose.target_pose.pose.position.x;
_goal.target_pose.pose.position.y = goal_pose.target_pose.pose.position.y ;
_goal.target_pose.pose.orientation.w= goal_pose.target_pose.pose.orientation.w;
_goal.target_pose.header.frame_id = "map";
ROS_INFO("goal is: x %.2f y %.2f z %.2f  \n ",_goal.target_pose.pose.position.x,_goal.target_pose.pose.position.y,_goal.target_pose.pose.position.z);
_delivered=false;
_ready=false;
donuts_=false;

}


void Pioneer::run(){
    boost::thread delivering_t( &Pioneer::delivering, this);
}


bool Pioneer::is_backHome(){
    return _ready;
}

void Pioneer::DoneCb(){
donuts_=true;

}

void Pioneer::delivering(){
    ros::Rate r(1);
    bool sent=false;
    ROS_INFO("delivering ready");
    while(ros::ok()){
       if(!_delivered){
       	if(!sent){
           _ac->sendGoal(_goal,boost::bind(&Pioneer::DoneCb, this),MoveBaseClient::SimpleActiveCallback(),MoveBaseClient::SimpleFeedbackCallback());
           
    	   ROS_INFO("goal sent");
    	   sent=true;
    	   }
            if(donuts_)
            {
            donuts_=false;
            _back=true;
            _delivered=true;
            sent=false;
            }
       }
        else if(_back){
        if(!sent){
           _ac->sendGoal(_base);
           sent=true;}
           
            if(donuts_)
            { 
            donuts_=false;
            _ready=true;
            _back=false;
            sent=false;
            }
       }
    r.sleep();
    }
    
}
/*
bool Pioneer::leave_the_box(){

}

void Pioneer::newBoxCb(){

 
}*/
//base pose for p3dx_1
move_base_msgs::MoveBaseGoal base_1;



//base pose for p3dx_2
move_base_msgs::MoveBaseGoal base_2;



//red box station
move_base_msgs::MoveBaseGoal red_station;




//blu box station
move_base_msgs::MoveBaseGoal blue_station;






/*
void manager(Pioneer* p1,Pioneer* p2){
   p1->go_to(red_station);
   p2->go_to(blue_station);
   int round_1=1;
   int round_2=1;
   while(ros::ok()){
       if(p1->is_backHome()){
           round_1++;
           ROS_INFO("giro completato numero %d",round_1);
           p1->go_to(blue_station);
       }
       if(p2->is_backHome()){
           round_2++;
           ROS_INFO("giro completato numero %d",round_2);
           p2->go_to(blue_station);
       }
   }

}*/

int main(int argc, char** argv){
    
    ros::init(argc, argv, "p3dx_fleet");
    ros::AsyncSpinner spinner(3);
    base_1.target_pose.pose.position.x = 2.0;
    base_1.target_pose.pose.position.y = -4.0;
    base_1.target_pose.pose.orientation.w = 6.0;
    base_1.target_pose.header.frame_id = "map";


    MoveBaseClient ac1("p3dx_1/move_base", true);
    Pioneer p3dx_1(&ac1,base_1);
    while(!ac1.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

    sleep(1);
    base_2.target_pose.pose.position.x = 2.0;
    base_2.target_pose.pose.position.y = -3.0;
    base_2.target_pose.pose.orientation.w = 6.0;
    base_2.target_pose.header.frame_id = "map";
    MoveBaseClient ac2("p3dx_2/move_base", true);
    Pioneer p3dx_2(&ac2,base_2);
     while(!ac2.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
    
    red_station.target_pose.pose.position.x = 2.0;
    red_station.target_pose.pose.position.y = 9.6;
    red_station.target_pose.pose.orientation.w = 6.0;
    red_station.target_pose.header.frame_id = "map";

    
    p3dx_2.go_to(red_station);

    
    blue_station.target_pose.pose.position.x = 2.0;
    blue_station.target_pose.pose.position.y = -9.6;
    blue_station.target_pose.pose.orientation.w = 6.0;
    blue_station.target_pose.header.frame_id = "map";

    p3dx_1.go_to(blue_station);
    
     
    
    int round_1=1;
    int round_2=1;
    spinner.start();
    p3dx_1.run();
    
    sleep(1);

    p3dx_2.run();
    
    sleep(2);
    ros::Rate looping(1);
    ROS_INFO("LOOP BEGIN, VALUE OF P3DX1_READY IS %d AND P3DX2_READY IS %d",p3dx_1.is_backHome(),p3dx_2.is_backHome());
    while(ros::ok()){

        if(p3dx_1.is_backHome()){
            round_1++;
            ROS_INFO("giro completato numero %d",round_1);
            p3dx_1.go_to(red_station);
        }
        if(p3dx_2.is_backHome()){
            round_2++;
            ROS_INFO("giro completato numero %d",round_2);
            p3dx_2.go_to(blue_station);
        }
        looping.sleep();
    }
    
return 0;
}
/*
prdx_1.go_to(red_station);
   prdx_2.go_to(blue_station);
   int round_1=1;
   int round_2=1;
   while(ros::ok()){
       if(p3dx_1.is_backHome()){
           round_1++;
           ROS_INFO("giro completato numero %d",round_1);
           prdx_1.go_to(blue_station);
       }
       if(p3dx_2.is_backHome()){
           round_2++;
           ROS_INFO("giro completato numero %d",round_2);
           prdx_2.go_to(blue_station);
       }
   }*/
