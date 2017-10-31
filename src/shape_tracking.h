#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "boost/thread.hpp"
#include <geometry_msgs/Point.h>

#include "ellipse_tracking.h"
#include "sphere_stereo_tracking.h"

#include "TooN/TooN.h"

using namespace std;
using namespace cv;
using namespace TooN;

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

  private:

    ros::NodeHandle _nh;
    ros::Subscriber _img_sub_l;
    ros::Subscriber _img_sub_r;
    ros::Publisher  _sphere_pub;
    ros::Publisher _c1_pub;
    ros::Publisher _c2_pub;

    //Input image
    Mat _src_l;
    Mat _src_r;
    bool _img_l_ready;
    bool _img_r_ready;

    //---Params
    string _img_topic_l;
    string _img_topic_r;
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
    //---

    ellipse_tracking *etrack;

};

void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);
