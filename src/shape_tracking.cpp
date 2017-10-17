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


void Dilation( int, void* ) {}


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
    high_b = max(high_b, low_b+1);
    setTrackbarPos("High B", "RGB", high_b);
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

  _img_ready = false;

  load_param( _img_topic, "/rodyman/left_camera/image_raw/rep", "img_left_topic" );
  load_param( _rate, 50, "rate" );
  load_param( _to_blur, false, "to_blur");
  load_param( _show_img_contounrs, false, "show_img_contounrs" );
  load_param( _off_x, 480, "off_x" );
  load_param( _off_y, 80, "off_y" );
  load_param( _set_RGB, false, "set_RGB");
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

  _img_sub = _nh.subscribe( _img_topic.c_str(), 0, &shape_tracking::cam_cb, this );

  empty_pub = _nh.advertise<std_msgs::Empty>("/out", 0);
  etrack = new ellipse_tracking();
}

void shape_tracking::tune_rgb_gain() {
  Mat img;
  while( !_img_ready ) {
    usleep(0.1*1e6);
  }
  cout << "_img_ready: " << _img_ready << endl;

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
    Mat img = _src;


    inRange(img,Scalar(low_b, low_g, low_r), Scalar(high_b, high_g, high_r), img);
    imshow( "RGB", img );
    waitKey(1);


    img.release();
  }

} //Toto: give possibility to save this params

void shape_tracking::tune_dilation() {
  Mat img;
  while( !_img_ready ) {
    usleep(0.1*1e6);
  }
  cout << "_img_ready: " << _img_ready << endl;


  namedWindow( "Dilation", CV_WINDOW_AUTOSIZE );
/*

  /// Create Dilation Trackbar
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
                    &dilation_elem, max_elem,
                    Dilation );

    createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo",
                    &dilation_size, max_kernel_size,
                    Dilation );
*/
  ros::Rate r(_rate);

  while(ros::ok()) {
    Mat img = _src;
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



void shape_tracking::cam_cb( sensor_msgs::Image msg ) {

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	_src = cv_ptr->image;
	_img_ready = true;
}


void shape_tracking::track_ellipses() {

  Mat img;
	while( !_img_ready ) {
		usleep(0.1*1e6);
	}
	cout << "_img_ready: " << _img_ready << endl;

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
  std_msgs::Empty emp;
  vector<vector<Point> > contours;

  int ellipse_min_c[2];
  int ellipse_max_c[2];
  ellipse_min_c[0] = ellipse_min_c[1] = 1000;
  ellipse_max_c[0] = ellipse_max_c[1] = -1000;

  while(ros::ok()) {

    img = _src;
    outer_ellipse.clear();
    inner_ellipse.clear();
    contours.clear();

    //---Get first ellipse
    Mat cropedImage = img(Rect( _off_x, _off_y, img.cols-_off_x, img.rows-_off_y));
    etrack->get_ellipse(cropedImage, _to_blur, low_rgb, high_rgb,
      _dilation_elem, _dilation_size, false, false, outer_ellipse, ellipse_center);

    /*
    contours.push_back( outer_ellipse );
    Scalar color = Scalar( 0, 0, 255 );
    drawContours( cropedImage, contours, 0, color, 2, 8 );
    circle( cropedImage, ellipse_center, 2, color, 2, 8 );
    imshow( "img", cropedImage );
    */
    //---

    //---Get second ellipse
    for(int pts=0; pts<outer_ellipse.size(); pts++ ) {
      ellipse_min_c[0] = (ellipse_min_c[0] > outer_ellipse[pts].x ) ? outer_ellipse[pts].x : ellipse_min_c[0];
      ellipse_min_c[1] = (ellipse_min_c[1] > outer_ellipse[pts].y ) ? outer_ellipse[pts].y : ellipse_min_c[1];
      ellipse_max_c[0] = (ellipse_max_c[0] < outer_ellipse[pts].x ) ? outer_ellipse[pts].x : ellipse_max_c[0];
      ellipse_max_c[1] = (ellipse_max_c[1] < outer_ellipse[pts].y ) ? outer_ellipse[pts].y : ellipse_max_c[1];
    }
    Mat ellipse_roi = cropedImage(Rect( ellipse_min_c[0]-_roi_off_x, ellipse_min_c[1]-_roi_off_y, (ellipse_max_c[0]-ellipse_min_c[0])+_roi_off_x*2, (ellipse_max_c[1]-ellipse_min_c[1])+_roi_off_y*2) );


    cvtColor( ellipse_roi, ellipse_roi, CV_BGR2GRAY );

    threshold( ellipse_roi, ellipse_roi, 220, 255, 1 );
    imshow( "ellipse_roi", ellipse_roi );

    //ellipse_roi = Scalar::all(255) - ellipse_roi;
    //imshow( "ellipse_roi", ellipse_roi );

//    etrack->get_ellipse(ellipse_roi, _to_blur, low_rgb, high_rgb,
//      _dilation_elem, _dilation_size, true, false, outer_ellipse, inner_ellipse_center);



//---Temp work only on roi!!!

//cvtColor( ellipse_roi, ellipse_roi, CV_BGR2GRAY );
//threshold( ellipse_roi, ellipse_roi, 220, 255, 1 );
//cv::imshow("ellipse_roi", ellipse_roi);


    //---

    waitKey(1);

    empty_pub.publish(emp);
    r.sleep();
  }

}

void shape_tracking::run() {

  if( _set_RGB )
    boost::thread tune_rgb_gain_t( &shape_tracking::tune_rgb_gain, this );
  else if( _set_dilation )
    boost::thread tune_dilation_t( &shape_tracking::tune_dilation, this );
  else
    boost::thread track_ellipses_t( &shape_tracking::track_ellipses, this );

  ros::spin();
}

int main( int argc, char** argv ) {

  ros::init( argc, argv, "shape_tracking");
  shape_tracking st;
  st.run();

  return 0;
} //main
