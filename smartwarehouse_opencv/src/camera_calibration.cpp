#include "obj_tracker.h"

using namespace std;
using namespace cv;

void box_tracking::cam_parameters( sensor_msgs::CameraInfo camera_info) {

    /*
     *  ROS topic data
     *  K = cameraMatrix
     *  D = distCoeffs
     *  R = Rettification
     *  P = Projection
     */

    if( _cam_info_first == false ) {

        ROS_INFO("Start camera parameters initialization...");
        //---resize calibration matrix
        _cam_cameraMatrix = new cv::Mat(3, 3, CV_64FC1);
        _cam_distCo = new cv::Mat(1, 5, CV_64FC1);
        _cam_R = new cv::Mat(3, 3, CV_64FC1);
        _cam_P = new cv::Mat(3, 4, CV_64FC1);
        //---

        //---K
        for(int i=0; i<3;i++) {
            for(int j=0; j<3; j++) {
                _cam_cameraMatrix->at<double>(i,j) = camera_info.K[3*i+j];

                cout << "[" << i << ", " << j << "]: " << _cam_cameraMatrix->at<double>(i,j) << endl;
            }
        }
        //---D
				if( camera_info.D.size() >= 5 ) {
	        for(int i=0; i<5;i++) {
            _cam_distCo->at<double>(0,i) = camera_info.D[i];
  	      }
				}
        //---R
        for(int i=0; i<3;i++) {
            for(int j=0; j<3; j++) {
                _cam_R->at<double>(i,j) = camera_info.R[3*i+j];
            }
        }
        //---P
        for(int i=0; i<3;i++) {
            for(int j=0; j<4; j++) {
                _cam_P->at<double>(i,j) = camera_info.P[4*i+j];
            }
        }
        _cam_info_first = true;

        ROS_INFO("...camera parameters initialization complete!");
    }

}
