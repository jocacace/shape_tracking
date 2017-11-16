#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "boost/thread.hpp"
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/CameraInfo.h"
#include <ros/package.h>
#include "ellipse_tracking.h"
#include "sphere_stereo_tracking.h"
#include "robohelper/robohelper.hpp"
#include "TooN/TooN.h"
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace std;
using namespace cv;
using namespace TooN;

//---3d output
typedef struct point {
  double x;
  double y;
  double z;
}point;

typedef struct points {
  point p1; //Center
  point p2; //Orientation
}points;

class shape_tracking {

  public:

    shape_tracking();
    void run();
    void cam_cb_l( sensor_msgs::Image imgl );
    void cam_cb_r( sensor_msgs::Image imgr );
    void track_ellipses_stereo();
    void track_ellipses();
    void tune_rgb_gain();
    void tune_dilation();
    void set_roi();
    void track_sphere();
    void cam1_parameters( sensor_msgs::CameraInfo camera_info);
    void cam2_parameters( sensor_msgs::CameraInfo camera_info);
    void img2space( int p_in[2], cv::Mat *_cameraMatrix, cv::Mat *_distCo, cv::Mat *_R, cv::Mat *_P, vector<double> & p_out );
    void dept_cb( sensor_msgs::Image depth );
    void save_images();


  private:

    ros::NodeHandle _nh;
    ros::Subscriber _img_sub_l;
    ros::Subscriber _img_sub_r;
    ros::Subscriber _depth_img_sub;
    ros::Publisher  _sphere_pub;
    ros::Publisher _c1_pub_l;
    ros::Publisher _c2_pub_l;
    ros::Publisher _c1_pub_r;
    ros::Publisher _c2_pub_r;

    ros::Subscriber _cam1_info_sub;
    ros::Subscriber _cam2_info_sub;


    //Input image
    Mat _src_l;
    Mat _src_r;
    Mat _depth_src;
    bool _img_l_ready;
    bool _depth_ready;
    bool _img_r_ready;
    bool _cam1_info_first;
    bool _cam2_info_first;
    bool _apply_roi;
    bool _use_depth;

    sensor_msgs::PointCloud2 pcl_depth;

    //---Params
    string _camera_left_info_topic;
    string _camera_right_info_topic;
    string _depth_img;
    string _img_topic_l;
    string _img_topic_r;
    string _cam_info_topic_l;
    string _cam_info_topic_r;
    int _port;
    string _address;

    string _task;
    int _off_x_l;
    int _off_y_l;
    int _rect_w_l;
    int _rect_h_l;
    int _off_x_r;
    int _off_y_r;
    int _rect_w_r;
    int _rect_h_r;
    int _rate;

    int _low_r_l;
    int _low_g_l;
    int _low_b_l;
    int _high_r_l;
    int _high_g_l;
    int _high_b_l;

    int _low_r_r;
    int _low_g_r;
    int _low_b_r;
    int _high_r_r;
    int _high_g_r;
    int _high_b_r;

    int _dilation_elem;
    int _dilation_size;
    int _th;
    int _roi_off_x;
    int _roi_off_y;
    bool _show_img_contounrs;
    bool _to_blur;
    bool _show_img_elaboration;
    bool _set_dilation;
    bool _set_RGB;
    bool _set_th;
    bool _set_roi;
    bool _stereo_cam;
    int _hc_param1;
    int _hc_param2;

    bool _track_orientation;
    //---

    ellipse_tracking *etrack;

    //---camera parameters
    cv::Mat *_cam1_cameraMatrix, *_cam1_distCo, *_cam1_R, *_cam1_P;
    cv::Mat *_cam2_cameraMatrix, *_cam2_distCo, *_cam2_R, *_cam2_P;
    Matx34d P0;
    Matx34d P10;
    //---


    int _output_data_socket;


};

void on_low_r_thresh_trackbar_r(int, void *);
void on_high_r_thresh_trackbar_r(int, void *);
void on_low_g_thresh_trackbar_r(int, void *);
void on_high_g_thresh_trackbar_r(int, void *);
void on_low_b_thresh_trackbar_r(int, void *);
void on_high_b_thresh_trackbar_r(int, void *);

void on_low_r_thresh_trackbar_l(int, void *);
void on_high_r_thresh_trackbar_l(int, void *);
void on_low_g_thresh_trackbar_l(int, void *);
void on_high_g_thresh_trackbar_l(int, void *);
void on_low_b_thresh_trackbar_l(int, void *);
void on_high_b_thresh_trackbar_l(int, void *);
