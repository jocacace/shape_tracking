#include "shape_tracking.h"

int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
int low_r=30;
int low_g=30;
int low_b=30;
int high_r=100;
int high_g=100;
int high_b=100;
int bin_th = 0;
int roi_x = 0;
int roi_y = 0;
int rect_h = 0;
int rect_w = 0;

void on_high_r_dilation_trackbar(int, void *) {}

void on_x_trackbar(int, void *) {
  setTrackbarPos("x","ROI", roi_x);
}

void on_y_trackbar(int, void *) {
  setTrackbarPos("y","ROI", roi_y);
}
void on_rw_trackbar(int, void *) {
  setTrackbarPos("rect_w","ROI", rect_w);
}

void on_rh_trackbar(int, void *) {
  setTrackbarPos("rect_h","ROI", rect_h);
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
    low_b= min(high_b-1, low_b);
    setTrackbarPos("Low B","RGB", low_b);
}
void on_high_b_thresh_trackbar(int, void *) {
    high_g = max(high_g, low_g+1);
    setTrackbarPos("High G", "RGB", high_g);
}

void on_thresh_trackbar(int, void *) {
    setTrackbarPos("bth","Binary threshold", bin_th);
}


//---Get params from ros parameter server
void load_param( string & p, string def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}
//---



shape_tracking::shape_tracking() {

  load_param( _img_topic_l, "/rodyman/left_camera/image_raw/rep", "img_left_topic" );
  load_param( _img_topic_r, "/rodyman/right_camera/image_raw/rep", "img_right_topic" );
  load_param( _rate, 50, "rate" );
  load_param( _to_blur, false, "to_blur");
  load_param( _show_img_contounrs, false, "show_img_contounrs" );
  load_param( _off_x_l, 480, "off_x_l" );
  load_param( _off_y_l, 80, "off_y_l" );
  load_param( _rect_h_l, 10, "rect_h_l" );
  load_param( _rect_w_l, 10, "rect_w_l" );

  load_param( _off_x_r, 480, "off_x_r" );
  load_param( _off_y_r, 80, "off_y_r");
  load_param( _rect_h_r, 10, "rect_h_r" );
  load_param( _rect_w_r, 10, "rect_w_r" );

  load_param( _set_RGB, false, "set_RGB");
  load_param( _set_th, false, "set_th");
  load_param( _set_dilation, false, "set_dilation");
  load_param( _show_img_elaboration, true, "show_image_elaboration");
  load_param( _low_r, 0, "low_r");
  load_param( _low_g, 0, "low_g");
  load_param( _low_b, 0, "low_b");
  load_param( _high_r, 0, "high_r");
  load_param( _high_g, 255, "high_g");
  load_param( _high_b, 255, "high_b");
  load_param( _dilation_elem, 0, "dilation_elem");
  load_param( _dilation_size, 7, "dilation_size");
  load_param( _roi_off_x, 0, "roi_off_x" );
  load_param( _roi_off_y, 0, "roi_off_y" );
  load_param( _th, 180, "th");
  load_param( _set_roi, false, "set_roi");
  load_param( _task, "sphere_tracking", "task");
  load_param( _stereo_cam, false, "stereo_cam");

  bin_th = _th;
  _img_sub_l = _nh.subscribe( _img_topic_l.c_str(), 0, &shape_tracking::cam_cb_l, this );
  if( _stereo_cam )
    _img_sub_r = _nh.subscribe( _img_topic_r.c_str(), 0, &shape_tracking::cam_cb_r, this );

  _c1_pub = _nh.advertise<geometry_msgs::Point>("/shape_tracking/ellipse_center", 0);
  _c2_pub = _nh.advertise<geometry_msgs::Point>("/shape_tracking/ellipse_orientation", 0);
  _sphere_pub = _nh.advertise<geometry_msgs::Point>("/shape_tracking/sphere_centre", 0);

  _img_l_ready = false;
  _img_r_ready = false;

  if( _task == "ellipse_tracking")
    etrack = new ellipse_tracking();
}

void shape_tracking::tune_rgb_gain() {
  Mat img;
  while( !_img_l_ready ) {
    usleep(0.1*1e6);
  }
  cout << "_img_l_ready: " << _img_l_ready << endl;

  low_r = _low_r;
  low_g = _low_g;
  low_b = _low_b;

  high_r = _high_r;
  high_g = _high_g;
  high_b = _high_b;

  namedWindow("RGB", WINDOW_NORMAL);
  //-- Trackbars to set thresholds for RGB values
  createTrackbar("Low R","RGB", &low_r, 255, on_low_r_thresh_trackbar);
  createTrackbar("High R","RGB", &high_r, 255, on_high_r_thresh_trackbar);
  createTrackbar("Low G","RGB", &low_g, 255, on_low_g_thresh_trackbar);
  createTrackbar("High G","RGB", &high_g, 255, on_high_g_thresh_trackbar);
  createTrackbar("Low B","RGB", &low_b, 255, on_low_b_thresh_trackbar);
  createTrackbar("High B","RGB", &high_b, 255, on_high_b_thresh_trackbar);

  ros::Rate r(_rate);

  while(ros::ok()) {
    Mat img = _src_l;


    inRange(img,Scalar(low_b, low_g, low_r), Scalar(high_b, high_g, high_r), img);
    imshow( "RGB", img );
    waitKey(1);


    img.release();
  }

} //Toto: give possibility to save this params

void shape_tracking::tune_dilation() {
  Mat img;
  while( !_img_l_ready ) {
    usleep(0.1*1e6);
  }
  cout << "_img_l_ready: " << _img_l_ready << endl;


  namedWindow( "Dilation", CV_WINDOW_AUTOSIZE );

  createTrackbar( "Kernel size:\n 2n +1", "Dilation",  &dilation_size, max_kernel_size, on_high_r_dilation_trackbar );


  ros::Rate r(_rate);

  while(ros::ok()) {
    Mat img = _src_l;
    _dilation_size = dilation_size;
    Mat element = getStructuringElement( _dilation_elem,
      Size( 2*_dilation_size + 1, 2*_dilation_size+1 ),
      Point( _dilation_size, _dilation_size ) );

    /// Apply the dilation operation
    inRange(img,Scalar(_low_b, _low_g, _low_r), Scalar(_high_b, _high_g, _high_r), img);
    dilate( img, img, element );
    imshow( "Dilation", img );
    waitKey(1);

    img.release();
  }
}



void shape_tracking::cam_cb_l( sensor_msgs::Image msg ) {

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	_src_l = cv_ptr->image;
	_img_l_ready = true;
}


void shape_tracking::cam_cb_r( sensor_msgs::Image msg ) {

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	_src_r = cv_ptr->image;
	_img_r_ready = true;
}


void shape_tracking::set_roi() {
  Mat img;
  while( !_img_l_ready ) {
    usleep(0.1*1e6);
  }
  cout << "_img_l_ready: " << _img_l_ready << endl;


  namedWindow("ROI", CV_WINDOW_AUTOSIZE);
  //-- Trackbars to set thresholds for RGB values
  roi_x = _off_x_l;
  roi_y = _off_y_l;
  rect_h = _rect_h_l;
  rect_w = _rect_w_l;

  img = _src_l;
  createTrackbar("x","ROI", &roi_x, img.cols-1, on_x_trackbar);
  createTrackbar("y","ROI", &roi_y, img.rows-1, on_y_trackbar);
  createTrackbar("rect_w","ROI", &rect_w, img.cols-1, on_rw_trackbar);
  createTrackbar("rect_h","ROI", &rect_h, img.rows-1, on_rh_trackbar);

  Scalar color = Scalar( 0, 0, 255 );

  ros::Rate r(_rate);
  while(ros::ok()) {
    Mat img = _src_l;

    _off_x_l = roi_x;
    _off_y_l = roi_y;
    _rect_w_l = rect_w;
    _rect_h_l = rect_h;

    Mat cropedImage = img(Rect( _off_x_l, _off_y_l, img.cols-_off_x_l, img.rows-_off_y_l));
    rectangle(cropedImage, Point(0,0), Point(_rect_w_l, _rect_h_l),color, 2, 8 );

    imshow( "ROI", cropedImage );
    waitKey(1);

    img.release();
  }
}

void shape_tracking::track_ellipses() {

  Mat img;
	while( !_img_l_ready ) {
		usleep(0.1*1e6);
	}
	cout << "_img_l_ready: " << _img_l_ready << endl;

  ros::Rate r(_rate);

  int low_rgb[3]; int high_rgb[3];
  low_rgb[0] = _low_r;
  low_rgb[1] = _low_g;
  low_rgb[2] = _low_b;
  high_rgb[0] = _high_r;
  high_rgb[1] = _high_g;
  high_rgb[2] = _high_b;

  vector<Point> outer_ellipse;
  vector<Point> inner_ellipse;


  Point ellipse_center;
  Point inner_ellipse_center;

  vector<vector<Point> > contours;

  int ellipse_min_c[2];
  int ellipse_max_c[2];

  ellipse_min_c[0] = ellipse_min_c[1] = 1000;
  ellipse_max_c[0] = ellipse_max_c[1] = -1000;

  geometry_msgs::Point p1, p2;
  bool invert_img = false;

  if( _set_th ) {
    namedWindow("Binary threshold", WINDOW_NORMAL);
    createTrackbar("bth","Binary threshold", &bin_th, 255, on_thresh_trackbar );
  }
  vector<Point> translated_c;
  Scalar color = Scalar( 0, 0, 255 );
  Point original_center_e1;
  Point original_center_e2;




  while(ros::ok()) {

    ellipse_min_c[0] = ellipse_min_c[1] = 1000;
    ellipse_max_c[0] = ellipse_max_c[1] = -1000;
    img = _src_l;
    outer_ellipse.clear();
    inner_ellipse.clear();
    contours.clear();

    //---Get first ellipse
    bool to_dilate = true;
    invert_img = false;
    //Mat cropedImage = img(Rect( _off_x, _off_y, img.cols-_off_x, img.rows-_off_y));
    Mat cropedImage = img(Rect( _off_x_l, _off_y_l, _rect_w_l, _rect_h_l));
    etrack->get_ellipse(cropedImage, _to_blur, low_rgb, high_rgb, invert_img, to_dilate, _dilation_elem, _dilation_size, false, false, outer_ellipse, ellipse_center);

    for(int pts=0; pts<outer_ellipse.size(); pts++ ) {
      ellipse_min_c[0] = (ellipse_min_c[0] > outer_ellipse[pts].x ) ? outer_ellipse[pts].x : ellipse_min_c[0];
      ellipse_min_c[1] = (ellipse_min_c[1] > outer_ellipse[pts].y ) ? outer_ellipse[pts].y : ellipse_min_c[1];
      ellipse_max_c[0] = (ellipse_max_c[0] < outer_ellipse[pts].x ) ? outer_ellipse[pts].x : ellipse_max_c[0];
      ellipse_max_c[1] = (ellipse_max_c[1] < outer_ellipse[pts].y ) ? outer_ellipse[pts].y : ellipse_max_c[1];
    }
    Mat ellipse_roi = cropedImage(Rect( ellipse_min_c[0]-_roi_off_x, ellipse_min_c[1]-_roi_off_y, (ellipse_max_c[0]-ellipse_min_c[0])+_roi_off_x*2, (ellipse_max_c[1]-ellipse_min_c[1])+_roi_off_y*2) );
    Mat ellipse_roi_tmp;


    if( _set_th ) {
      _th = bin_th;
      cvtColor( ellipse_roi, ellipse_roi_tmp, CV_BGR2GRAY );
      threshold( ellipse_roi_tmp, ellipse_roi_tmp, _th, 255, 1 );
      imshow("Binary threshold",ellipse_roi_tmp);
      waitKey(1);
    }
    else {
      cvtColor( ellipse_roi, ellipse_roi_tmp, CV_BGR2GRAY );
      threshold( ellipse_roi_tmp, ellipse_roi_tmp, _th, 255, 1 );
      to_dilate = false;
      invert_img = true;
      etrack->get_ellipse(ellipse_roi_tmp, _to_blur, invert_img, to_dilate, _dilation_elem, _dilation_size, false, false, inner_ellipse, inner_ellipse_center);
    }



    original_center_e1.x = (ellipse_center.x + _off_x_l  );
    original_center_e1.y = (ellipse_center.y + _off_y_l  );
    original_center_e2.x = (inner_ellipse_center.x + _off_x_l + ellipse_min_c[0] );
    original_center_e2.y = (inner_ellipse_center.y + _off_y_l + ellipse_min_c[1] );


    if( _show_img_elaboration ) {
      translated_c.clear();
      Point conts;
      for(int i=0; i<outer_ellipse.size(); i++) {
        conts.x = outer_ellipse[i].x + _off_x_l ;
        conts.y = outer_ellipse[i].y + _off_y_l;
        translated_c.push_back(conts);
      }
      contours.clear();
      contours.push_back( translated_c );
      drawContours( img, contours, 0, color, 2, 8 );

      circle( img, original_center_e1, 2, color, 2, 8 );

      circle( img, original_center_e2, 2, color, 2, 8 );
      imshow( "img", img );
      waitKey(1);

    }
    p1.x = original_center_e1.x;
    p1.y = original_center_e1.y;
    p2.x = original_center_e2.x;
    p2.y = original_center_e2.y;

    _c1_pub.publish( p1 );
    _c2_pub.publish( p2 );

    r.sleep();
  }

}

void shape_tracking::track_sphere() {

  while( !_img_l_ready || !_img_r_ready )
    usleep(0.1*1e6);
  ROS_INFO("Image left and right ready!");

  ros::Rate r(_rate);

  vpHomogeneousMatrix cMo;

  Mat img_l;
  sphere_stero_tracking sst(true, 120, 40);
  cv::Mat left, right;
  left = cv::Mat::zeros(_src_l.size(), CV_8UC3);
  right = cv::Mat::zeros(_src_r.size(), CV_8UC3);

  left = _src_l;
  right = _src_r;
  sst.init("",left,right);

  Vector<3> p0, p0cam1, p0cam2;
  Point p_img_left;
  Point p_img_right;

  p0 = Zeros;
  p0cam1 = Zeros;
  p0cam2 = Zeros;
  geometry_msgs::Point sp_c;

  while(ros::ok()) {
    left = _src_l;
    right = _src_r;

    Mat cleft = left(Rect( _off_x_l, _off_y_l, _rect_w_l, _rect_h_l));
    Mat cright = right(Rect( _off_x_r, _off_y_r, _rect_w_r, _rect_h_r));

    //Track
    sst.track(left, right, p0, p0cam1, p0cam2, p_img_left, p_img_right );
    Scalar color = Scalar( 0, 0, 255 );

    if(_show_img_elaboration) {
      circle( left, p_img_left, 2, color, 2, 8 );
      circle( right, p_img_right, 2, color, 2, 8 );
      imshow( "right", right);
      waitKey(10);
    }

    sp_c.x = p0[0];
    sp_c.y = p0[1];
    sp_c.z = p0[2];
    _sphere_pub.publish( sp_c );

    r.sleep();
  }
}

void shape_tracking::run() {

  if( _set_RGB )
    boost::thread tune_rgb_gain_t( &shape_tracking::tune_rgb_gain, this );
  else if( _set_dilation )
    boost::thread tune_dilation_t( &shape_tracking::tune_dilation, this );
  else if( _set_roi )
    boost::thread tune_roi_t( &shape_tracking::set_roi, this );
  else if( _task == "ellipse_tracking")
    boost::thread track_ellipses_t( &shape_tracking::track_ellipses, this );
  else if( _task == "sphere_tracking")
    boost::thread track_ellipses_t( &shape_tracking::track_sphere, this );

  ros::spin();
}

int main( int argc, char** argv ) {

  ros::init( argc, argv, "shape_tracking");
  shape_tracking st;
  st.run();

  return 0;
} //main
