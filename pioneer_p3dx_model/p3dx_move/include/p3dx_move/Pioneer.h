#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Pioneer {
    public:
        Pioneer(MoveBaseClient* ac);
        Pioneer(MoveBaseClient* ac_spec,const move_base_msgs::MoveBaseGoal starting_position);
        void go_to(const move_base_msgs::MoveBaseGoal goal_position);
        void run();
        bool is_backHome();
        
        
    private:
        MoveBaseClient* _ac;
        move_base_msgs::MoveBaseGoal _base;
        move_base_msgs::MoveBaseGoal _goal;
        bool _ready;
        bool _delivered;
        bool _back;
        void delivering();
        //ros::NodeHandle _nh;
        //ros::Subscriber _new_box;
        //ros::Publisher  _box_done;
        //box_queue
};
