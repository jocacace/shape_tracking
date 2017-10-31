#include "shape_tracking.h"

//save camera parameters in openCV structs
void shape_tracking::cam1_parameters( sensor_msgs::CameraInfo camera_info) {

    /*
     *  ROS topic data
     *  K = cameraMatrix
     *  D = distCoeffs
     *  R = Rettification
     *  P = Projection
     */

    if( _cam1_info_first == false ) {

        ROS_INFO("Start camera parameters initialization...");
        //---resize calibration matrix
        _cam1_cameraMatrix = new cv::Mat(3, 3, CV_64FC1);
        _cam1_distCo = new cv::Mat(1, 5, CV_64FC1);
        _cam1_R = new cv::Mat(3, 3, CV_64FC1);
        _cam1_P = new cv::Mat(3, 4, CV_64FC1);
        //---

        //---K
        for(int i=0; i<3;i++) {
            for(int j=0; j<3; j++) {
                _cam1_cameraMatrix->at<double>(i,j) = camera_info.K[3*i+j];
            }
        }
        //---D
				if( camera_info.D.size() >= 5 ) {
	        for(int i=0; i<5;i++) {
            _cam1_distCo->at<double>(0,i) = camera_info.D[i];
  	      }
				}
        //---R
        for(int i=0; i<3;i++) {
            for(int j=0; j<3; j++) {
                _cam1_R->at<double>(i,j) = camera_info.R[3*i+j];
            }
        }
        //---P
        for(int i=0; i<3;i++) {
            for(int j=0; j<4; j++) {
                _cam1_P->at<double>(i,j) = camera_info.P[4*i+j];
            }
        }
        _cam1_info_first = true;

        ROS_INFO("...camera parameters initialization complete!");
    }

}


void shape_tracking::cam2_parameters( sensor_msgs::CameraInfo camera_info) {
  /*
   *  ROS topic data
   *  K = cameraMatrix
   *  D = distCoeffs
   *  R = Rettification
   *  P = Projection
   */

  if( _cam2_info_first == false ) {

      ROS_INFO("Start camera parameters initialization...");
      //---resize calibration matrix
      _cam2_cameraMatrix = new cv::Mat(3, 3, CV_64FC1);
      _cam2_distCo = new cv::Mat(1, 5, CV_64FC1);
      _cam2_R = new cv::Mat(3, 3, CV_64FC1);
      _cam2_P = new cv::Mat(3, 4, CV_64FC1);
      //---

      //---K
      for(int i=0; i<3;i++) {
          for(int j=0; j<3; j++) {
              _cam2_cameraMatrix->at<double>(i,j) = camera_info.K[3*i+j];
          }
      }
      //---D
      if( camera_info.D.size() >= 5 ) {
        for(int i=0; i<5;i++) {
          _cam2_distCo->at<double>(0,i) = camera_info.D[i];
        }
      }
      //---R
      for(int i=0; i<3;i++) {
          for(int j=0; j<3; j++) {
              _cam2_R->at<double>(i,j) = camera_info.R[3*i+j];
          }
      }
      //---P
      for(int i=0; i<3;i++) {
          for(int j=0; j<4; j++) {
              _cam2_P->at<double>(i,j) = camera_info.P[4*i+j];
          }
      }
      _cam2_info_first = true;

      ROS_INFO("...camera parameters initialization complete!");
  }
}
