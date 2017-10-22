#include "sphere_stereo_tracking.h"


/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
vpMeEllipse2::vpMeEllipse2()
{

}

/*!
  Copy constructor.
*/
vpMeEllipse2::vpMeEllipse2(const vpMeEllipse2 &meellipse)
{
  iPc0 = meellipse.iPc0;
}

/*!
  Basic destructor.
*/
vpMeEllipse2::~vpMeEllipse2()
{

}


void
vpMeEllipse2::updateList()
{
  vpMeSite p_me;
  double theta;
  for(std::list<vpMeSite>::iterator it=list.begin(); it!=list.end(); ++it){
    p_me = *it;
    vpImagePoint iP;
    p_me.ifloat += iPc.get_i() - iPc0.get_i();
    p_me.jfloat += iPc.get_j() - iPc0.get_j();
    *it = p_me;
  }
}

void
vpMeEllipse2::initTrackingCircle(vpImage<unsigned char> &I) {

  const unsigned int n=5 ;
  vpImagePoint iP[n];

  std::cout << " semi axis " << a << std::endl;

  iP[0].set_i(iPc.get_i() + a);
  iP[0].set_j(iPc.get_j());
  iP[1].set_i(iPc.get_i() - a);
  iP[1].set_j(iPc.get_j());
  iP[2].set_i(iPc.get_i());
  iP[2].set_j(iPc.get_j() + a);
  iP[3].set_i(iPc.get_i());
  iP[3].set_j(iPc.get_j() - a);
  iP[4].set_i(iPc.get_i() + 1);
  iP[4].set_j(iPc.get_j() - a);

  iP1 = iP[0];
  iP2 = iP[n-1];

  initTracking(I, n, iP) ;

}


/*!
  Display the ellipse.

  \warning To effectively display the ellipse a call to
  vpDisplay::flush() is needed.

  \param I : RGB Image in which the ellipse appears.
  \param col : Color of the displayed ellipse.
 */

void
vpMeEllipse2::display(const vpImage<vpRGBa> &I, vpColor col)
{
	vpMeEllipse::display(I,iPc,a,b,e,0,2*M_PI,col,2);
}


sphere_stero_tracking::sphere_stero_tracking(bool disp_, int param1_, int param2_) {
  flag0 = true;
  isLost = false;

  _param_1 = param1_;
  _param_2 = param2_;
  _disp = disp_;
}


void sphere_stero_tracking::load_conf() {
  //TMP! hardcoded!
  vpCameraParameters cameraLeft( 743.0730868231319, 741.3507300477314, 512.0658141069166, 256.0660499493296);
  cam1 = cameraLeft;

  vpCameraParameters cameraRight( 743.7026312116332, 741.8680387429362,534.1757633664125, 245.9841950877609 );
  cam2 = cameraRight;

  me1.sample_step = 2;
  me1.ntotal_sample = 3000;
  me1.setMaskSize(5);
  me1.setMaskNumber(180);
  me1.range = 5;
  me1.mu1 = 0.5;
  me1.mu2 = 0.5;
  me1.threshold = 40000;

  dp = 1;
  
  param_1 = _param_1; //120
  param_2 = _param_2; //40

  min_radius = 5;
  max_radius = 100;

  Kleft = Mat(cam1.get_K().getRows(), cam1.get_K().getRows(), cv::DataType<double>::type);
  Kleftinv = Mat( cam1.get_K_inverse().getRows(), cam1.get_K_inverse().getRows(), cv::DataType<double>::type);

  Kright = Mat(cam2.get_K().getRows(), cam2.get_K().getRows(), cv::DataType<double>::type);
  Krightinv = Mat( cam2.get_K_inverse().getRows(), cam2.get_K_inverse().getRows(), cv::DataType<double>::type);

  for (int k = 0; k < Kleft.rows; k++)
    for (int l = 0; l < Kleft.cols; l++) {
      Kleft.at<double>(k, l) = cam1.get_K()[k][l];
      Kleftinv.at<double>(k, l) = cam1.get_K_inverse()[k][l];
      Kright.at<double>(k, l) = cam2.get_K()[k][l];
      Krightinv.at<double>(k, l) = cam2.get_K_inverse()[k][l];
    }

  distcoeff1 = cv::Mat(1,4,cv::DataType<double>::type);
  distcoeff1.at<double>(0,0) = -0.1339858023201154;
  distcoeff1.at<double>(0,1) = 0.1507880965066347;
  distcoeff1.at<double>(0,2) = -0.00456996529265559;
  distcoeff1.at<double>(0,3) = 0.0009810721446667698;

  distcoeff2 = cv::Mat(1,4,cv::DataType<double>::type);
  distcoeff2.at<double>(0,0) = -0.1420526347764651;
  distcoeff2.at<double>(0,1) = 0.1660345338305312;
  distcoeff2.at<double>(0,2) = -0.003983095275512944;
  distcoeff2.at<double>(0,3) = 0.002559983330240779;

  c2Mc1[0][0] = 0.9997656194949723;
  c2Mc1[1][0] = -0.0192813567040456;
  c2Mc1[2][0] = 0.009845575629974224;

  c2Mc1[0][1] = 0.01924301738740872;
  c2Mc1[1][1] = 0.9998069378888094;
  c2Mc1[2][1] = 0.003974069857265075;

  c2Mc1[0][2] = -0.009920300280841923;
  c2Mc1[1][2] = -0.003783679829728314;
  c2Mc1[2][2] = 0.9999436341160856;

  c2Mc1[0][3] = -0.1478065127138684;
  c2Mc1[1][3] = 0.001063827926624086;
  c2Mc1[2][3] = -0.004848910650938509;
}



void sphere_stero_tracking::init(const std::string &filename, cv::Mat &left, cv::Mat &right) {

  load_conf();

  vpImageConvert::convert(left, Irgbleft);
  vpImageConvert::convert(right, Irgbright);

  lcinit = left.clone();
  rcinit = right.clone();

  vpImageConvert::convert(Irgbleft, Ileft);
  vpImageConvert::convert(Irgbright, Iright);

  Ileft.quarterSizeImage(Ileftsmall);
  Iright.quarterSizeImage(Irightsmall);

  vpPoint p1;
  p1.setWorldCoordinates(0,0,0);
  radius = 0.0425;
  /*
  tracker.setStereoCameraParameters(cam1,cam2,c2Mc1);
  tracker.setMovingEdgeStereo(me1,me1);
  tracker.addSphereStereo(p1, radius);
  */

  Eleft.setMe(&me1) ;
  Eleft.setCircle(true) ;
  Eleft.setDisplay(vpMeSite::RANGE_RESULT) ;

  me2 = me1;
  Eright.setMe(&me2) ;
  Eright.setCircle(true) ;
  Eright.setDisplay(vpMeSite::RANGE_RESULT) ;

  int width = Irgbright.getWidth();
  int height = Irgbright.getHeight();

  rectangleleft.x = (int) ceil(width * 0.1);
  rectangleleft.y = (int) ceil(height * 0.1);
  rectangleleft.width = width - 2 * rectangleleft.x;
  rectangleleft.height = height - 2 * rectangleleft.y;

  rectangleright.x = (int) ceil(width * 0.1);
  rectangleright.y = (int) ceil(height * 0.1);
  rectangleright.width = width - 2 * rectangleright.x;
  rectangleright.height = height - 2 * rectangleright.y;

  Eleft.setMe(&me1) ;
  Eleft.setDisplay(vpMeSite::RANGE_RESULT) ;

  Eright.setMe(&me2) ;
  Eright.setDisplay(vpMeSite::RANGE_RESULT) ;

  iter00 = 0;

  Matx34d P0(1,	0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0);

  Matx34d P20(c2Mc1[0][0],    c2Mc1[0][1], c2Mc1[0][2], c2Mc1[0][3],
              c2Mc1[1][0],    c2Mc1[1][1], c2Mc1[1][2], c2Mc1[1][3],
              c2Mc1[2][0],    c2Mc1[2][1], c2Mc1[2][2], c2Mc1[2][3]);

  P = P0;
  P2 = P20;


}


void sphere_stero_tracking::extractCircles(cv::Mat &src, vector<Vec3f> &circles_) {

  cv::Mat src_gray, src_gray0;

  src_gray = src;
  src_gray0 = src;

  cv::GaussianBlur( src_gray, src_gray0, Size(9, 9), 2, 2 );

  vector<Vec3f> circles;

  //res 1280x1024
  // ballplate
  /*if (res) cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 40, 60, 0, 70);
  else cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 40, 50, 0, 80 );*/

  /* if (res) cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 100, 60, 5, 100);
  else cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 100, 60, 5, 100 );*/

  /*if (res) cv::HoughCircles( src_gray0, circles, CV_HOUGH_GRADIENT, dp, src_gray.rows/8, param_1, param_2, min_radius, max_radius);
  else */
  //cv::HoughCircles( src_gray0, circles, CV_HOUGH_GRADIENT, dp, src_gray.rows/8, param_1, param_2, min_radius, max_radius);
  cv::HoughCircles( src_gray0, circles, CV_HOUGH_GRADIENT, dp, src_gray.rows/8, param_1, param_2, min_radius, max_radius);

  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ ) {

    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // circle center
    circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
    // circle outline
    circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
  }
  circles_ = circles;

}

//void sphere_stero_tracking::track( cv::Mat &left, cv::Mat &right, vpHomogeneousMatrix &cMo_) {
void sphere_stero_tracking::track( cv::Mat &left, cv::Mat &right, Vector<3> & p0, Vector<3> & p0cam1, Vector<3> & p0cam2, Point & p0img_left, Point & p0img_right) {

  cv::Mat lc0,rc0,lc1,rc1,lg1,rg1,lccrop,rccrop, lc0crop,rc0crop,lc1crop,rc1crop;

  lccrop = left(rectangleleft);
  rccrop = right(rectangleright);

  cv::Mat crop_foreground_left,foreground0_left, crop_bin_left;
  cv::Mat crop_foreground_right,foreground0_right, crop_bin_right;
  cv::Mat lgcrop,rgcrop;

  double t01;

  vpImageConvert::convert(left, Irgbleft);
  vpImageConvert::convert(right, Irgbright);

  cv::cvtColor(lccrop, lgcrop,CV_BGR2GRAY);
  cv::cvtColor(rccrop, rgcrop,CV_BGR2GRAY);

  lg1 = cv::Mat::zeros(lcinit.size(),CV_8UC1);
  rg1 = cv::Mat::zeros(rcinit.size(),CV_8UC1);

  cv::Mat dst_roig = lg1(rectangleleft);
  lgcrop.copyTo(dst_roig);

  dst_roig = rg1(rectangleright);
  rgcrop.copyTo(dst_roig);

  vpImageConvert::convert(lg1, Ileft);
  vpImageConvert::convert(rg1, Iright);

  Ileft.quarterSizeImage(Ileftsmall);
	Iright.quarterSizeImage(Irightsmall);

  vector<Vec3f> circlesLeft,circlesRight;

  extractCircles(lgcrop, circlesLeft);
  extractCircles(rgcrop, circlesRight);

  int gapRect;
  int gapCenter;
  gapRect = 100;
  gapCenter = 120;

  if (circlesLeft.size() > 0 ) {
    for( size_t i = 0; i < circlesLeft.size(); i++ ) {
      if (flag0 || isLost) {
        Eleft.iPc0 = Eleft.iPc;
        Eleft.iPc.set_u(cvRound(circlesLeft[i][0]) + rectangleleft.x);
        Eleft.iPc.set_v(cvRound(circlesLeft[i][1]) + rectangleleft.y);
        Eleft.a = cvRound(circlesLeft[i][2]);
        Eleft.b = cvRound(circlesLeft[i][2]);
        Eleft.updateList();
      }
      else {
        if (sqrt(pow(cvRound(circlesLeft[i][0])+ rectangleleft.x -Eleft.iPc.get_u(), 2) + pow(cvRound(circlesLeft[i][1]) + rectangleleft.y -Eleft.iPc.get_v(), 2))<gapRect && cvRound(circlesLeft[i][2])- Eleft.getA() < gapCenter) {
          Eleft.iPc0 = Eleft.iPc;
          Eleft.iPc.set_u(cvRound(circlesLeft[i][0]) + rectangleleft.x);
          Eleft.iPc.set_v(cvRound(circlesLeft[i][1]) + rectangleleft.y);
          Eleft.a = cvRound(circlesLeft[i][2]);
          Eleft.b = cvRound(circlesLeft[i][2]);
          Eleft.updateList();
        }
      }
    }
    flagLeft = false;
  }

  if (circlesRight.size() > 0 ) {
    for( size_t i = 0; i < circlesRight.size(); i++ ) {
      if (flag0 || isLost) {
        Eright.iPc0 = Eright.iPc;
        Eright.iPc.set_u(cvRound(circlesRight[i][0]) + rectangleright.x);
        Eright.iPc.set_v(cvRound(circlesRight[i][1]) + rectangleright.y);
        Eright.a = cvRound(circlesRight[i][2]);
        Eright.b = cvRound(circlesRight[i][2]);
        Eleft.updateList();
      }
      else {
        if (sqrt(pow(cvRound(circlesRight[i][0])+ rectangleright.x -Eright.iPc.get_u(), 2) + pow(cvRound(circlesRight[i][1]) + rectangleright.y -Eright.iPc.get_v(), 2))<gapRect && cvRound(circlesRight[i][2])- Eright.getA() < gapCenter) {
          Eright.iPc0 = Eright.iPc;
          Eright.iPc.set_u(cvRound(circlesRight[i][0]) + rectangleright.x);
          Eright.iPc.set_v(cvRound(circlesRight[i][1]) + rectangleright.y);
          Eright.a = cvRound(circlesRight[i][2]);
          Eright.b = cvRound(circlesRight[i][2]);
          Eright.updateList();
        }
      }
    }
    flagRight = false;
  }

  if (!flagLeft && !flagRight && flag0) {
    Eleft.initTrackingCircle(Ileft);
    Eright.initTrackingCircle(Iright);

    flagLeft = true;
    flag0 = false;
  }


  if(!flag0) {
    double angle,anglel,angler;
    int surface,surfacel,surfacer;
    vpImagePoint ip,ipl,ipr;
    Eleft.track(Ileft);
		Eright.track(Iright);

    Eright.iPc.set_ij(Eright.iPc.get_i(),Eright.iPc.get_j());
    Eleft.iPc.set_ij(Eleft.iPc.get_i(),Eleft.iPc.get_j());
    int cornRect;
		int widthRect;
		cornRect = 50;
		widthRect = 100; //TODO: param!
		double x1_n, y1_n, x2_n, y2_n;
		double rad1, rad2;
		rectangleleft.x = Eleft.iPc.get_u() - Eleft.getA() - cornRect;
		rectangleleft.y = Eleft.iPc.get_v() - Eleft.getA() - cornRect;
		rectangleleft.height = 2*Eleft.getA() + widthRect;
		rectangleleft.width = 2*Eleft.getA() + widthRect;

		rectangleright.x = Eright.iPc.get_u() - Eright.getA() - cornRect;
		rectangleright.y = Eright.iPc.get_v() - Eright.getA() - cornRect;
		rectangleright.height = 2*Eright.getA() + widthRect;
		rectangleright.width = 2*Eright.getA() + widthRect;

    p0img_left.x = Eleft.iPc.get_u();
    p0img_left.y = Eleft.iPc.get_v();

    p0img_right.x = Eright.iPc.get_u();
    p0img_right.y = Eright.iPc.get_v();

		vpPixelMeterConversion::convertPoint(cam1,Eleft.iPc, x1_n,y1_n);
		vpPixelMeterConversion::convertPoint(cam2,Eright.iPc, x2_n,y2_n);

		rad2 = Eright.getA();
		rad1 = Eleft.getA();

		vpHomogeneousMatrix c1Mo,c2Mo, c12Mo;

	  c2Mo[2][3] = (cam2.get_px()*radius/rad2 );
	  c2Mo[0][3] = (c2Mo[2][3]*x2_n);
	  c2Mo[1][3] = (c2Mo[2][3]*y2_n);
	  c1Mo[2][3] = (cam1.get_px()*radius/(rad1) );
	  c1Mo[0][3] = (c1Mo[2][3]*x1_n);
	  c1Mo[1][3] = (c1Mo[2][3]*y1_n);
	  cMo = c1Mo;
	  c12Mo = c2Mc1.inverse()*c2Mo;
	  iter0++;

	  vpHomogeneousMatrix cMor;
	  cMor = cMo;
	  cMor[2][3] = cMo[2][3] ;//- 0.04;

	  c2Mo = c2Mc1*cMo;

    p0 = makeVector(cMo[0][3], cMo[1][3],cMo[2][3] );
    p0cam1 = makeVector(cMo[0][3]/cMo[2][3], cMo[1][3]/cMo[2][3], 0 );
    p0cam2 = makeVector(cMo2[0][3]/cMo2[2][3], cMo2[1][3]/cMo2[2][3], 0 );

		cv::Mat pnts3D(1,1,CV_64FC4);
		cv::Mat cam0pnts(1,1,CV_64FC2);
		cv::Mat cam1pnts(1,1,CV_64FC2);

		cam0pnts.at<cv::Vec2d>(0,0)[0] = (float)Eleft.iPc.get_u();
		cam0pnts.at<cv::Vec2d>(0,0)[1] = (float)Eleft.iPc.get_v();

		cam1pnts.at<cv::Vec2d>(0,0)[0] = (float)Eright.iPc.get_u();
		cam1pnts.at<cv::Vec2d>(0,0)[1] = (float)Eright.iPc.get_v();

    cv::Mat pt_set1_pt,pt_set2_pt;
    Mat pt_3d_h(1,1,CV_32FC4);
    vector<Point3f> pt_3d;
    cv::undistortPoints(cam0pnts, pt_set1_pt, Kleft, distcoeff1);
    cv::undistortPoints(cam1pnts, pt_set2_pt, Kright, distcoeff2);

    cv::triangulatePoints(Mat(P),Mat(P2), pt_set1_pt,pt_set2_pt,pt_3d_h);
    cv::triangulatePoints(Kleft*Mat(P),Kright*Mat(P2),cam0pnts,cam1pnts,pnts3D);

    vpHomogeneousMatrix cMo0,cMo02;
    cMo0 = cMo;

    cMo0[0][3] = pnts3D.at<cv::Vec4d>(0,0)[0]/pnts3D.at<cv::Vec4d>(0,0)[3];
		cMo0[1][3] = pnts3D.at<cv::Vec4d>(0,0)[1]/pnts3D.at<cv::Vec4d>(0,0)[3];
		cMo0[2][3] = pnts3D.at<cv::Vec4d>(0,0)[2]/pnts3D.at<cv::Vec4d>(0,0)[3];

		cMo0[0][3] = pt_3d_h.at<cv::Vec4d>(0,0)[0]/pt_3d_h.at<cv::Vec4d>(0,0)[3];
		cMo0[1][3] = pt_3d_h.at<cv::Vec4d>(0,0)[1]/pt_3d_h.at<cv::Vec4d>(0,0)[3];
		cMo0[2][3] = pt_3d_h.at<cv::Vec4d>(0,0)[2]/pt_3d_h.at<cv::Vec4d>(0,0)[3];

		x1_n = cMo0[0][3]/cMo0[2][3];
		y1_n = cMo0[1][3]/cMo0[2][3];

		cMo02 = c2Mc1*cMo0;

		x2_n = cMo02[0][3]/cMo02[2][3];
    y2_n = cMo02[1][3]/cMo02[2][3];

		cMo = cMo0;
		cMo2 = cMo02;

    if (Eleft.iPc.get_j() > left.cols - 30 || Eleft.iPc.get_j() < 30 || Eright.iPc.get_j() > right.cols - 30 || Eright.iPc.get_j() < 30 || Eleft.iPc.get_i() > left.rows - 30 || Eleft.iPc.get_i() < 30 || Eright.iPc.get_i() > right.rows - 30 || Eright.iPc.get_i() < 30) {

      cMo[0][3] = 0;
      cMo[1][3] = 0;
      cMo[2][3] = 0;

      cMo2[0][3] = 0;
      cMo2[1][3] = 0;
      cMo2[2][3] = 0;
    }

		vpMeterPixelConversion::convertPoint(cam1,x1_n,y1_n,center1);
		vpMeterPixelConversion::convertPoint(cam2,x2_n,y2_n,center2);
  	double rectx, recty;
  	rectx = 0;
  	recty = 0;
    rectangleleft.x = Eleft.getCenter().get_u() - Eleft.getA() - cornRect;
  	rectangleleft.y = Eleft.getCenter().get_v() - Eleft.getA() - cornRect;

    if (rectangleleft.x <= 0 ) {
      rectx = rectangleleft.x;
      rectangleleft.x = 1;
    }
    if (rectangleleft.y <= 0 ) {
      recty = rectangleleft.y;
      rectangleleft.y = 1;
    }

    rectangleleft.height = 2*Eleft.getA() + widthRect - recty;
    rectangleleft.width = 2*Eleft.getA() + widthRect - rectx;

    if (rectangleleft.x + rectangleleft.width >=  Ileft.getWidth()) {
      rectangleleft.width -= rectangleleft.x + rectangleleft.width -Ileft.getWidth();
    }
    if (rectangleleft.y + rectangleleft.height >= Ileft.getHeight() ) {
      rectangleleft.height -= rectangleleft.y + rectangleleft.height-Ileft.getHeight();
    }

    rectangleright.x = Eright.getCenter().get_u() - Eright.getA() - cornRect;
    rectangleright.y = Eright.getCenter().get_v() - Eright.getA() - cornRect;

    if (rectangleright.x <= 0 ) {
      rectx = rectangleright.x;
      rectangleright.x = 1;
    }
    if (rectangleright.y <= 0 ) {
      recty = rectangleright.y;
      rectangleright.y = 1;
    }

    rectangleright.height = 2*Eright.getA() + widthRect - recty;
    rectangleright.width = 2*Eright.getA() + widthRect - rectx;

    if (rectangleright.x + rectangleright.width >=  Iright.getWidth()) {
      rectangleright.width -= rectangleright.x + rectangleright.width -Iright.getWidth();
    }

    if (rectangleright.y + rectangleright.height >= Iright.getHeight() ) {
      rectangleright.height -= rectangleright.y + rectangleright.height-Iright.getHeight();
    }

    //cMo_ = cMo;
    p0 = makeVector(cMo[0][3], cMo[1][3],cMo[2][3] );
  }

  if( _disp ) {
    imshow("lgcrop", lgcrop);
    imshow("rgcrop", rgcrop);
    waitKey(1);
  }


}
