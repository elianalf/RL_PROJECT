#include "obj_tracker.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


using namespace std;
using namespace cv;
using namespace tf;

bool first_col;

int low_r = 5;
int low_g = 0;
int low_b = 0;
int high_r = 225;
int high_g = 1;
int high_b = 1;

double rgbd_roll;
double rgbd_yaw;
double rgbd_pitch;
double rgbd_x;
double rgbd_y;
double rgbd_z;
Eigen::Matrix<double,3,3> rotation_matrix3;
Eigen::Matrix<double,3,3> rotation_matrix3_2;

 void box_tracking::_get_model_name(double px, double py, double pz, string& box_name){
    //** Find the model name of the nearest box to the box in position (px,py,pz) **
   double min_d=1000;
   int index =-1;
   for(int j=0; j<init_pose.size(); j++){
      double dx = px - init_pose[j].position.x;
      double dy = py - init_pose[j].position.y;
      double dz = pz - init_pose[j].position.z;
      double d = sqrt(pow(dx,2)+ pow(dy,2)+ pow(dz,2));
      if( d < min_d){
         min_d=d;
         index=j;
      }
   }
   
   if(index==-1){
      cout<<"No model name found"<<endl;
       box_name = "";}
   else{
       box_name=model_names[index];
       cout<<"Box name "<<box_name<<endl;}
 }

void box_tracking::model_states_cb( gazebo_msgs::ModelStates m_states){
   size_t m_size = m_states.name.size();
   model_names.resize(m_size);
   init_pose.resize(m_size);
   for(int i=0; i<m_size; i++){
      model_names[i] = m_states.name[i];
      init_pose[i] = m_states.pose[i];
      //cout<<m_states.name[i]<<endl;
   }
}


void on_low_r_thresh_trackbar(int, void *) {
    low_r = min(high_r-1, low_r);
    setTrackbarPos("Low R","RGB", low_r);
}
void on_high_r_thresh_trackbar(int, void *) {
    high_r = max(high_r, low_r+1);
    setTrackbarPos("High R", "RGB", high_r);
}
void on_low_g_thresh_trackbar(int, void *) {
    low_g = min(high_g-1, low_g);
    setTrackbarPos("Low G","RGB", low_g);
}
void on_high_g_thresh_trackbar(int, void *) {
    high_g = max(high_g, low_g+1);
    setTrackbarPos("High G", "RGB", high_g);
}
void on_low_b_thresh_trackbar(int, void *) {
    low_b = min(high_b-1, low_b);
    setTrackbarPos("Low B","RGB", low_b);
}
void on_high_b_thresh_trackbar(int, void *) {
    high_g = max(high_g, low_g+1);
    setTrackbarPos("High B", "RGB", high_b);
}

//LOAD TOPIC
void load_param( string & p, string def, string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

//LOAD THRESHOLD AND RATE
void load_param( int & p, int def, string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}
//LOAD COORDINATE COMPONENT
void load_param( double & p, double def, string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}



//TUNING TRUE OR FALSE
void load_param( bool & p, bool def, string name ) {
  ros::NodeHandle n_param;
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}



box_tracking::box_tracking() {
    
    first_col=true;
    string _img_topic;
    string _depth_img_topic;
    string _cam_info_topic;

    load_param( _img_topic, "/depth_camera/depth_camera/image_raw", "img_topic" );
    load_param( _depth_img_topic, "/depth_camera/depth/disparity", "depth_img_topic"); 
    load_param( _cam_info_topic, "/depth_camera/depth_camera/camera_info", "camera_info_topic" );
    load_param( _rate, 20, "rate" );
    load_param( _set_RGB, false, "set_RGB");
    
    load_param( _low_r_1, 5, "low_r_1");
    load_param( _low_g_1, 0, "low_g_1");
    load_param( _low_b_1, 0, "low_b_1");
    load_param( _high_r_1, 225, "high_r_1");
    load_param( _high_g_1, 1, "high_g_1");
    load_param( _high_b_1, 1, "high_b_1");
    
    load_param( _low_r_2, 0, "low_r_2");
    load_param( _low_g_2, 0, "low_g_2");
    load_param( _low_b_2, 5, "low_b_2");
    load_param( _high_r_2, 1, "high_r_2");
    load_param( _high_g_2, 1, "high_g_2");
    load_param( _high_b_2, 225, "high_b_2");
    double def=0.0;
    load_param( rgbd_roll, def , "rgbd_roll" );
    load_param( rgbd_yaw, def , "rgbd_yaw" );
    load_param( rgbd_pitch, def , "rgbd_pitch" );
    load_param( rgbd_x, def , "rgbd_x" );
    load_param( rgbd_y,def , "rgbd_y" );
    load_param( rgbd_z, def , "rgbd_z" );
    
    
    Eigen::Vector3f ea(rgbd_yaw, rgbd_pitch ,rgbd_roll);
    Eigen::Vector3f ea_camera_rgbd(-1.57, 0 ,-1.57);
    
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    rotation_matrix3_2 = Eigen::AngleAxisd( ea_camera_rgbd[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd( ea_camera_rgbd[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd( ea_camera_rgbd[2], Eigen::Vector3d::UnitX());
    cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;
    
    _gazebo_sub = _nh.subscribe( "/gazebo/model_states", 0, &box_tracking::model_states_cb, this );
    _img_sub = _nh.subscribe( _img_topic.c_str(), 0, &box_tracking::cam_cb, this );
    _depth_img_sub = _nh.subscribe( _depth_img_topic.c_str(),0, &box_tracking::depth_cb, this );
    _cam_info_sub = _nh.subscribe(_cam_info_topic.c_str(), 0, &box_tracking::cam_parameters, this);
  
    _img_ready = _depth_ready = _cam_info_first = false;
  
}


void box_tracking::cam_cb( sensor_msgs::ImageConstPtr img ) {

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); 
        _src = cv_ptr->image;
        _img_ready = true;
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void box_tracking::depth_cb( sensor_msgs::ImageConstPtr depth ) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1); 
        _depth_src = cv_ptr->image;
        _depth_ready = true;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void box_tracking::tune_rgb_gain() {

    Mat img;
    while( !_img_ready ) { 
        usleep(0.1*1e6);
    }
    
    low_r = _low_r_1;
    low_g = _low_g_1;
    low_b = _low_b_1;

    high_r = _high_r_1;
    high_g = _high_g_1;
    high_b = _high_b_1;

    namedWindow("RGB", WINDOW_NORMAL);
    
    createTrackbar("Low R", "RGB", &low_r,  255, on_low_r_thresh_trackbar);
    createTrackbar("High R","RGB", &high_r, 255, on_high_r_thresh_trackbar);
    createTrackbar("Low G", "RGB", &low_g,  255, on_low_g_thresh_trackbar);
    createTrackbar("High G","RGB", &high_g, 255, on_high_g_thresh_trackbar);
    createTrackbar("Low B", "RGB", &low_b,  255, on_low_b_thresh_trackbar);
    createTrackbar("High B","RGB", &high_b, 255, on_high_b_thresh_trackbar);

    ros::Rate r(_rate);

    while(ros::ok()) {
        Mat img = _src;

        if( img.empty() )
            continue;
      
        inRange(img,Scalar(low_b, low_g, low_r), Scalar(high_b, high_g, high_r), img);
        imshow( "RGB", img );
        
        waitKey(1);
      
        img.release();
    }
} 




void box_tracking::track_rectangle() {
    ros::Rate r(_rate);
    Mat img;
    Mat depth;
    while( !_img_ready || !_depth_ready || !_cam_info_first) {
        usleep(0.1*1e6);
    }  
   
   actionlib::SimpleActionClient<smart_warehouse_2::box_posAction> a_c("abb_robot_server", true);
   cout << "Waiting for server" << endl;
   a_c.waitForServer(); //will wait for infinite time
   cout << "Server is ready" << endl;
   smart_warehouse_2::box_posGoal pos_goal;
   smart_warehouse_2::box_posResult result;
   int low_rgb[3]; int high_rgb[3];  
   vector<Point> rectangle_center_blue;
   vector<Point> rectangle_center_red;
   double cx, cy, fx_inv, fy_inv;
   vector<double> cx_c, cy_c, cz_c;
   float zd_c1;
   float zd_c2;
   Matrix3x3 Rot_matrix_;
    
   while(ros::ok()) {
      bool box_moved = false;
      img = _src;
      depth = _depth_src;
      
	//scopriamo se ci sono sia rossi che blu
	int red_exist=0;
	int blue_exist=0;
	int total_exist=0;
      //elaboro i Rossi
      low_rgb[0] = _low_r_1;
      low_rgb[1] = _low_g_1;
      low_rgb[2] = _low_b_1;
      high_rgb[0] = _high_r_1;
      high_rgb[1] = _high_g_1;
      high_rgb[2] = _high_b_1;
      if(get_rectangle( img, low_rgb, high_rgb,  rectangle_center_red)){
		red_exist=1;
      }


	low_rgb[0] = _low_r_2;
	low_rgb[1] = _low_g_2;
	low_rgb[2] = _low_b_2;
	high_rgb[0] = _high_r_2;
	high_rgb[1] = _high_g_2;
	high_rgb[2] = _high_b_2;
	if(get_rectangle( img, low_rgb, high_rgb,  rectangle_center_blue)){
		blue_exist=2;
	}
	total_exist=red_exist+blue_exist;
	switch (total_exist){
	case 1 : //only reds exist;
	cout<<"only REDS";
	cx_c.resize(rectangle_center_red.size());  
        cy_c.resize(rectangle_center_red.size());  
        cz_c.resize(rectangle_center_red.size());
        for(int p=0;p<rectangle_center_red.size();p++){  
        cx = _cam_cameraMatrix->at<double>(0,2);
        cy = _cam_cameraMatrix->at<double>(1,2);
        fx_inv = 1.0 / _cam_cameraMatrix->at<double>(0,0);
        fy_inv = 1.0 / _cam_cameraMatrix->at<double>(1,1);
        
        zd_c1 = depth.at<float>(rectangle_center_red[p].y,rectangle_center_red[p].x);
        cx_c[p] = (zd_c1) * ( (rectangle_center_red[p].x - cx) * fx_inv );
        cy_c[p] = (zd_c1) * ( (rectangle_center_red[p].y - cy) * fy_inv );
        cz_c[p] = zd_c1;
        
        }
          pos_goal.color = "red";
        break;
	case 2 : //only blues exist;
	cout<<"only BLUES";
	cx_c.resize(rectangle_center_blue.size());  
	cy_c.resize(rectangle_center_blue.size());  
	cz_c.resize(rectangle_center_blue.size());
	for(int p=0;p<rectangle_center_blue.size();p++){  
	cx = _cam_cameraMatrix->at<double>(0,2);
	cy = _cam_cameraMatrix->at<double>(1,2);
	fx_inv = 1.0 / _cam_cameraMatrix->at<double>(0,0);
	fy_inv = 1.0 / _cam_cameraMatrix->at<double>(1,1);

	zd_c1 = depth.at<float>(rectangle_center_blue[p].y,rectangle_center_blue[p].x);
	cx_c[p] = (zd_c1) * ( (rectangle_center_blue[p].x - cx) * fx_inv );
	cy_c[p] = (zd_c1) * ( (rectangle_center_blue[p].y - cy) * fy_inv );
	cz_c[p] = zd_c1;
	
	}
          pos_goal.color = "blue";
	break;
	case 3 : //both exist; choose  the most numerous
	cout<<"BOTH BLUES AND REDS";
        if(rectangle_center_blue.size()>rectangle_center_red.size()){ //choose blue
        cx_c.resize(rectangle_center_blue.size());  
	cy_c.resize(rectangle_center_blue.size());  
	cz_c.resize(rectangle_center_blue.size());
	for(int p=0;p<rectangle_center_blue.size();p++){  
	cx = _cam_cameraMatrix->at<double>(0,2);
	cy = _cam_cameraMatrix->at<double>(1,2);
	fx_inv = 1.0 / _cam_cameraMatrix->at<double>(0,0);
	fy_inv = 1.0 / _cam_cameraMatrix->at<double>(1,1);

	zd_c1 = depth.at<float>(rectangle_center_blue[p].y,rectangle_center_blue[p].x);
	cx_c[p] = (zd_c1) * ( (rectangle_center_blue[p].x - cx) * fx_inv );
	cy_c[p] = (zd_c1) * ( (rectangle_center_blue[p].y - cy) * fy_inv );
	cz_c[p] = zd_c1;
	}
	pos_goal.color = "blue";
        cout<<" \n FIRST BLUES";
        }
        else{ //if less or equal choose red
        cx_c.resize(rectangle_center_red.size());  
        cy_c.resize(rectangle_center_red.size());  
        cz_c.resize(rectangle_center_red.size());
        for(int p=0;p<rectangle_center_red.size();p++){  
        cx = _cam_cameraMatrix->at<double>(0,2);
        cy = _cam_cameraMatrix->at<double>(1,2);
        fx_inv = 1.0 / _cam_cameraMatrix->at<double>(0,0);
        fy_inv = 1.0 / _cam_cameraMatrix->at<double>(1,1);
        
        zd_c1 = depth.at<float>(rectangle_center_red[p].y,rectangle_center_red[p].x);
        cx_c[p] = (zd_c1) * ( (rectangle_center_red[p].x - cx) * fx_inv );
        cy_c[p] = (zd_c1) * ( (rectangle_center_red[p].y - cy) * fy_inv );
        cz_c[p] = zd_c1;
       
        }
          pos_goal.color = "red";
          cout<<"\n FIRST REDS \n ";
        }
        break;
        }
        
        //once you choose the color, choose the nearest box
        int chosen=0;
        for(int p=1;p<rectangle_center_red.size();p++){
        if(cz_c[p]>cz_c[chosen]){chosen=p;}
        }
        
        

        
	Eigen::Matrix<double,3,1> traslation(rgbd_x,rgbd_y-0.02,rgbd_z-0.12);
		
	Eigen::Matrix<double,3,1> pose(cx_c[chosen],cy_c[chosen],cz_c[chosen]);
        pose=rotation_matrix3*rotation_matrix3_2*pose+traslation;
        Rot_matrix_.setRPY(3.14, 0, -1.57);
       	
       	
       	/* OLD TRASFORMATION
         double px=(Rot_matrix_[0].x() * cx_c[chosen])+(Rot_matrix_[0].y() * cy_c[chosen])+(Rot_matrix_[0].z()* cz_c[chosen]) - 1.5;
          double py=(Rot_matrix_[1].x() * cx_c[chosen])+(Rot_matrix_[1].y() * cy_c[chosen])+(Rot_matrix_[1].z()* cz_c[chosen]) -0.02;
          double pz=(Rot_matrix_[2].x() * cx_c[chosen])+(Rot_matrix_[2].y() * cy_c[chosen])+(Rot_matrix_[2].z()* cz_c[chosen]) + 1.88; 
         
        // ** Send the goal to the action server **
        ROS_INFO("after x y z %.2f %.2f %.2f",px,py,pz);*/
        cout<<" \n pose with eigen : "<<pose;
        
        
        
         double px=pose[0];         
         double py=pose[1];         
         double pz=pose[2]; 
           
          _get_model_name(px,py, pz, pos_goal.box_name);
          pos_goal.x_box =px;
          pos_goal.y_box = py;
          pos_goal.z_box = pz;
           cout<<"Sending position in world frame: "<<px<<" "<<py<<" "<<pz<<endl;
          a_c.sendGoal(pos_goal);
          while(!box_moved){
             usleep(1);
             if (a_c.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
		              cout << "Box successfully moved!" << endl;
			           r.sleep();
			           box_moved = true;
		       }
		       else if(a_c.getState() == actionlib::SimpleClientGoalState::ABORTED) {
		          cout<<"The goal was ABORTED by the action server due to some failure"<<endl;
               usleep(2000000);
               cout<<"Resending goal"<<endl;
               a_c.sendGoal(pos_goal);
               a_c.waitForResult();
		         
		       }
           }
           
       
        r.sleep();
    }
}

void box_tracking::run() {

  if( _set_RGB ){
    boost::thread tune_rgb_gain_t( &box_tracking::tune_rgb_gain, this );}
  else {
    boost::thread track_rectangle_t( &box_tracking::track_rectangle, this );}
 
  ros::spin();
}




int main(int argc, char** argv) {
    ros::init( argc, argv, "object_tracker");
    //3.1 Euler angle converted to rotation matrix
    box_tracking bt;
    bt.run();

    return 0;

}




