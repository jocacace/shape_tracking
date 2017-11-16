#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpMath.h>
#include <visp/vpVideoReader.h>
#include <visp/vpParseArgv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <visp/vpDebug.h>
#include <visp/vpConfig.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include <visp/vpMeEllipse.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

#include <visp/vpMeTracker.h>
#include <visp/vpMeSite.h>
#include <visp/vpImagePoint.h>

#include <visp/vpImage.h>
#include <visp/vpColor.h>

#include <visp/vpMeEllipse.h>

#include <math.h>
#include <list>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpMe.h>
#include <visp/vpPoint.h>

#include "TooN/TooN.h"
using namespace std;
using namespace cv;
using namespace TooN;

class VISP_EXPORT vpMeEllipse2 : public vpMeEllipse
{
public:
  vpMeEllipse2() ;
  vpMeEllipse2(const vpMeEllipse2 &meellipse) ;
  virtual ~vpMeEllipse2() ;

  void display(const vpImage<vpRGBa>&I, vpColor col) ;

  //! Former coordinates of the ellipse center.
  vpImagePoint iPc0;
  int getListSize(){return list.size();}
  void updateList();
  void initTrackingCircle(vpImage<unsigned char> &I);

};

class sphere_stero_tracking {

public:
  sphere_stero_tracking(bool disp_, int, int);
  void init(const std::string &filename, cv::Mat &left, cv::Mat &right);
  void track( cv::Mat &left, cv::Mat &right, TooN::Vector<3> & p0, TooN::Vector<3> & p0cam1, TooN::Vector<3> & p0cam2, Point & p0img_left, Point & p0img_right );
  void extractCircles(cv::Mat &src, vector<Vec3f> &circles_);
  void load_conf();

private:

  //! Vector of scale level to use for the multi-scale tracking.
  std::vector<bool> _scales;
  //! Pyramid of image associated to the current image. Thcis pyramid is computed in the init() and in the track() methods.
  std::vector< const vpImage<unsigned char>* > Ipyramid, Ipyramid1, Ipyramid2;

  vpImage<unsigned char> Ileft, Iright, Ileftsmall, Irightsmall;
  vpImage<vpRGBa> Irgbleft, Irgbright;
  vpImage<vpRGBa>  Ibinright, Isegright, Ibinleft, Isegleft;
  vpDisplayX displayLeft, displayRight, displayBinLeft, displayBinRight, displayRGBLeft, displayRGBRight ;
  vpMeEllipse2 Eleft,Eright;
  cv::Rect rectangleleft, rectangleright, rectangleleftInit, rectanglerightInit;

  Matx34d P,P1,P2;
  int iter00;
  vpHomogeneousMatrix c2Mc1,c1Mc2, cMo2;
  cv::Mat lcinit, rcinit;
  cv::Mat foreground_left;
  cv::Mat foreground_right;

  vpHomogeneousMatrix cMo;
  int iter0;
  double param_1; // 200: Upper threshold for the internal Canny edge detector
  double param_2; // 200: Upper threshold for the internal Canny edge detector
  int min_radius; //0: Minimum radio to be detected. If unknown, put zero as default.
  int max_radius; //0: Maximum radius to be detected. If unknown, put zero as default
  double dp ;//= 1  The inverse ratio of resolution
  bool flagLeft, flagRight, flagMbt, flag0;
  bool isLost;

  vpCameraParameters cam1, cam2;
  vpMe me1, me2;

  cv::Mat Kleft, Kleftinv;
  cv::Mat Kright, Krightinv;

  cv::Mat distcoeff1, distcoeff2;
  double radius;
  vpImagePoint center1,center2;

  bool _disp;
  int _param_1;
  int _param_2;
};
