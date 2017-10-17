#include "ros/ros.h"
#include "ellipse_tracking.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "boost/thread.hpp"

#include <std_msgs/Empty.h>

using namespace std;
using namespace cv;

class shape_tracking {

  public:
    shape_tracking();
    void run();
    void cam_cb( sensor_msgs::Image imgl );
    void track_ellipses();
    void tune_rgb_gain();
    void tune_dilation();
  private:
    ros::NodeHandle _nh;
    ros::Subscriber _img_sub;

    //Input image
    Mat _src;
    bool _img_ready;

    //---Params
    string _img_topic;
    int _off_x;void Dilation( int, void* ) {}

    int _off_y;
    int _rate;
    bool _show_img_contounrs;
    bool _to_blur;
    bool _show_img_elaboration;
    bool _set_dilation;
    bool _set_RGB;
    int _low_r;
    int _low_g;
    int _low_b;
    int _high_r;
    int _high_g;
    int _high_b;
    int _dilation_elem;
    int _dilation_size;

    int _roi_off_x;
    int _roi_off_y;
    //---

    ellipse_tracking *etrack;


    //Tmp for disp_debug
    ros::Publisher empty_pub;

};

void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);
void Dilation( int, void* );
