#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include "smart_warehouse_2/box_posAction.h"
#include "boost/thread.hpp"
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/CameraInfo.h"
#include <ros/package.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "tf/LinearMath/Matrix3x3.h"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
using namespace std;
using namespace cv;


class box_tracking {

  public:

    box_tracking();
    void run();
    void cam_cb( sensor_msgs::ImageConstPtr img );
    void depth_cb( sensor_msgs::ImageConstPtr depth );
    void track_rectangle();
    void tune_rgb_gain();
    void set_roi();
    void cam_parameters( sensor_msgs::CameraInfo camera_info);
    void img2space( int p_in[2], cv::Mat *_cameraMatrix, cv::Mat *_distCo, cv::Mat *_R, cv::Mat *_P, vector<double> & p_out );
   void model_states_cb(gazebo_msgs::ModelStates m_states);
   void _get_model_name(double px, double py, double pz, string& model_name );
   
  private:

    ros::NodeHandle _nh;
    ros::Subscriber _img_sub;
    ros::Subscriber _depth_img_sub;
    ros::Subscriber _cam_info_sub;
    ros::Subscriber _gazebo_sub;
    bool _set_RGB;
    cv::Mat *_cam_cameraMatrix, *_cam_distCo, *_cam_R, *_cam_P;
    std::vector<string> model_names;
    std::vector<geometry_msgs::Pose> init_pose;
   
    int _low_r_1;
    int _low_g_1;
    int _low_b_1;
    int _high_r_1;
    int _high_g_1;
    int _high_b_1;
    
    int _low_r_2;
    int _low_g_2;
    int _low_b_2;
    int _high_r_2;
    int _high_g_2;
    int _high_b_2;
    
    Mat _src;
    Mat _depth_src;
    bool _img_ready;
    bool _depth_ready;
    bool _cam_info_first;
    int _rate;


};


bool get_rectangle( Mat img,  int low_rgb[3], int high_rgb[3],  vector< Point> & center );


