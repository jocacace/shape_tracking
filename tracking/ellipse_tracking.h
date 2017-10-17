#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
using namespace std;
using namespace cv;

class ellipse_tracking {
  public:
    ellipse_tracking();
    void get_ellipse(Mat img, bool blur_img, int low_rgb[3], int high_rgb[3], int dilation_elem, int dilation_size, bool disp_debug, bool disp_output, vector< Point> &, Point & c);

  private:


};
