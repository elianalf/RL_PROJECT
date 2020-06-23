#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "boost/thread.hpp"
#include "geometry_msgs/Pose.h"
#include "smart_warehouse_2/box_posAction.h"
#include "gazebo_msgs/ModelState.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "tf/LinearMath/Matrix3x3.h"

#include "std_msgs/String.h"
  
using namespace std;
using namespace tf;

class PICK_PLACE_TASK {

  public:

    PICK_PLACE_TASK(string name_);
    void run();
    bool moveit_abb(double px, double py, double pz);
    void executeCB( const smart_warehouse_2::box_posGoalConstPtr &goal );
    void preemptCB();
   void abort_handler();
  private:
   
   ros::NodeHandle _n;
   ros::Publisher topic_pub;
   ros::Publisher Pioneer_pub;
   actionlib::SimpleActionServer<smart_warehouse_2::box_posAction> a_s; 
   string action_name;
   Matrix3x3 Rot_matrix_;


};

