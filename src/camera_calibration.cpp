#include "shape_tracking.h"

void shape_tracking::img2space( int p_in[2], cv::Mat *_cameraMatrix, cv::Mat *_distCo, cv::Mat *_R, cv::Mat *_P, vector<double> & p_out ) {

  std::vector<cv::KeyPoint> pixel;
  pixel.resize(1); //2 points to convert

  pixel[0].pt.x = p_in[0];
  pixel[0].pt.y = p_in[1];

  cv::Mat src = cv::Mat(1,1,CV_32FC2);
  src.at<cv::Vec2f>(0,0)[0] = p_in[0];
  src.at<cv::Vec2f>(0,0)[1] = p_in[1];

  cv::Mat dst = cv::Mat(1, pixel.size(), CV_32FC2);
  cv::undistortPoints(src, dst, *_cameraMatrix, *_distCo);

  p_out.resize(3);
  p_out[0] = dst.at<cv::Vec2f>(0,0)[0];
  p_out[1] = dst.at<cv::Vec2f>(0,0)[1];
  p_out[2] = 1.0;

}


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

                cout << "[" << i << ", " << j << "]: " << _cam1_cameraMatrix->at<double>(i,j) << endl;
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
