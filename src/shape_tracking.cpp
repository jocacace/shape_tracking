#include "shape_tracking.h"

int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

int low_r_l=30;
int low_g_l=30;
int low_b_l=30;
int high_r_l=100;
int high_g_l=100;
int high_b_l=100;

int low_r_r=30;
int low_g_r=30;
int low_b_r=30;
int high_r_r=100;
int high_g_r=100;
int high_b_r=100;


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

void on_low_r_thresh_trackbar_l(int, void *) {
    low_r_l = min(high_r_l-1, low_r_l);
    setTrackbarPos("Low R","RGB", low_r_l);
}
void on_high_r_thresh_trackbar_l(int, void *) {
    high_r_l = max(high_r_l, low_r_l+1);
    setTrackbarPos("High R", "RGB", high_r_l);
}
void on_low_g_thresh_trackbar_l(int, void *) {
    low_g_l = min(high_g_l-1, low_g_l);
    setTrackbarPos("Low G","RGB", low_g_l);
}
void on_high_g_thresh_trackbar_l(int, void *) {
    high_g_l = max(high_g_l, low_g_l+1);
    setTrackbarPos("High G", "RGB", high_g_l);
}
void on_low_b_thresh_trackbar_l(int, void *) {
    low_b_l= min(high_b_l-1, low_b_l);
    setTrackbarPos("Low B","RGB", low_b_l);
}
void on_high_b_thresh_trackbar_l(int, void *) {
    high_g_l = max(high_g_l, low_g_l+1);
    setTrackbarPos("High G", "RGB", high_g_l);
}

void on_low_r_thresh_trackbar_r(int, void *) {
    low_r_r = min(high_r_r-1, low_r_r);
    setTrackbarPos("Low R","RGB", low_r_r);
}
void on_high_r_thresh_trackbar_r(int, void *) {
    high_r_r = max(high_r_r, low_r_r+1);
    setTrackbarPos("High R", "RGB", high_r_r);
}
void on_low_g_thresh_trackbar_r(int, void *) {
    low_g_r = min(high_g_r-1, low_g_r);
    setTrackbarPos("Low G","RGB", low_g_r);
}
void on_high_g_thresh_trackbar_r(int, void *) {
    high_g_r = max(high_g_r, low_g_r+1);
    setTrackbarPos("High G", "RGB", high_g_r);
}
void on_low_b_thresh_trackbar_r(int, void *) {
    low_b_r= min(high_b_r-1, low_b_r);
    setTrackbarPos("Low B","RGB", low_b_r);
}
void on_high_b_thresh_trackbar_r(int, void *) {
    high_g_r = max(high_g_r, low_g_r+1);
    setTrackbarPos("High G", "RGB", high_g_r);
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
  load_param( _depth_img, "/camera/depth/image_raw", "depth_img_topic");

  load_param( _cam_info_topic_l, "/rodyman/left_camera/camera_info", "camera_left_info_topic" );
  load_param( _cam_info_topic_r, "/rodyman/right_camera/camera_info", "camera_right_info_topic" );

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
  load_param( _low_r_l, 0, "low_r_l");
  load_param( _low_g_l, 0, "low_g_l");
  load_param( _low_b_l, 0, "low_b_l");
  load_param( _high_r_l, 0, "high_r_l");
  load_param( _high_g_l, 255, "high_g_l");
  load_param( _high_b_l, 255, "high_b_l");

  load_param( _low_r_r, 0, "low_r_r");
  load_param( _low_g_r, 0, "low_g_r");
  load_param( _low_b_r, 0, "low_b_r");
  load_param( _high_r_r, 0, "high_r_r");
  load_param( _high_g_r, 255, "high_g_r");
  load_param( _high_b_r, 255, "high_b_r");

  load_param( _dilation_elem, 0, "dilation_elem");
  load_param( _dilation_size, 7, "dilation_size");
  load_param( _roi_off_x, 0, "roi_off_x" );
  load_param( _roi_off_y, 0, "roi_off_y" );
  load_param( _th, 180, "th");
  load_param( _set_roi, false, "set_roi");
  load_param( _task, "sphere_tracking", "task");
  load_param( _stereo_cam, false, "stereo_cam");
  load_param( _use_depth, false, "use_depth");

  load_param( _apply_roi, false, "apply_roi");

  load_param( _track_orientation, false, "track_orientation");

  //temp
  load_param( _port, 9090, "port");
  load_param( _address, "192.168.1.1", "address");

  bin_th = _th;
  _img_sub_l = _nh.subscribe( _img_topic_l.c_str(), 0, &shape_tracking::cam_cb_l, this );
  if( _stereo_cam )
    _img_sub_r = _nh.subscribe( _img_topic_r.c_str(), 0, &shape_tracking::cam_cb_r, this );

  if( _use_depth) {
    _depth_img_sub = _nh.subscribe( _depth_img.c_str(),0, &shape_tracking::dept_cb, this );
  }

  _c1_pub_l = _nh.advertise<geometry_msgs::Point>("/shape_tracking/cam1/ellipse_center", 0);
  _c2_pub_l = _nh.advertise<geometry_msgs::Point>("/shape_tracking/cam1/ellipse_orientation", 0);

  _c1_pub_r = _nh.advertise<geometry_msgs::Point>("/shape_tracking/cam2/ellipse_center", 0);
  _c2_pub_r = _nh.advertise<geometry_msgs::Point>("/shape_tracking/cam2/ellipse_orientation", 0);

  _sphere_pub = _nh.advertise<geometry_msgs::Point>("/shape_tracking/sphere_centre", 0);

  _img_l_ready = false;
  _img_r_ready = false;
  _cam1_info_first = false;
  _cam2_info_first = false;

  _cam1_info_sub = _nh.subscribe(_cam_info_topic_l.c_str(), 0, &shape_tracking::cam1_parameters, this);
  if( _stereo_cam )
    _cam2_info_sub = _nh.subscribe(_cam_info_topic_r.c_str(), 0, &shape_tracking::cam2_parameters, this);

  if( _task == "ellipse_tracking")
    etrack = new ellipse_tracking();

  Matx34d P0_t(1,	0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0);

 P0 = P0_t;

 Matx34d P10_t(0.999, -0.0179, 0.0015, -0.1508,
              0.0179, 0.99, 0.0011, 0.003,
              -0.0015, -0.0011, 1.0, 0.0005);
 P10 = P10_t;
 robohelper::create_socket(robohelper::string2char(_address), _port, &_output_data_socket );



}

void shape_tracking::dept_cb( sensor_msgs::Image msg ) {

  cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	_depth_src = cv_ptr->image;
	_depth_ready = true;
}

void shape_tracking::tune_rgb_gain() {
  Mat img;
   while( !_img_l_ready ) {
     usleep(0.1*1e6);
   }
   cout << "_img_l_ready: " << _img_l_ready << endl;

   low_r_l = _low_r_l;
   low_g_l = _low_g_l;
   low_b_l = _low_b_l;

   high_r_l = _high_r_l;
   high_g_l = _high_g_l;
   high_b_l = _high_b_l;

   namedWindow("RGB", WINDOW_NORMAL);
   //-- Trackbars to set thresholds for RGB values
   createTrackbar("Low R","RGB", &low_r_l, 255, on_low_r_thresh_trackbar_l);
   createTrackbar("High R","RGB", &high_r_l, 255, on_high_r_thresh_trackbar_l);
   createTrackbar("Low G","RGB", &low_g_l, 255, on_low_g_thresh_trackbar_l);
   createTrackbar("High G","RGB", &high_g_l, 255, on_high_g_thresh_trackbar_l);
   createTrackbar("Low B","RGB", &low_b_l, 255, on_low_b_thresh_trackbar_l);
   createTrackbar("High B","RGB", &high_b_l, 255, on_high_b_thresh_trackbar_l);

   ros::Rate r(_rate);

   while(ros::ok()) {
     Mat img = _src_l;

     if( img.empty() )
       continue;

     inRange(img,Scalar(low_b_l, low_g_l, low_r_l), Scalar(high_b_l, high_g_l, high_r_l), img);
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
    inRange(img,Scalar(_low_b_l, _low_g_l, _low_r_l), Scalar(_high_b_l, _high_g_l, _high_r_l), img);
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

  if( _use_depth ) {
    while( !_depth_ready ) {
  		usleep(0.1*1e6);
  	}
    cout << "_depth_ready: " << _depth_ready << endl;
  }

  ros::Rate r(_rate);

  int low_rgb[3]; int high_rgb[3];
  low_rgb[0] = _low_r_l;
  low_rgb[1] = _low_g_l;
  low_rgb[2] = _low_b_l;
  high_rgb[0] = _high_r_l;
  high_rgb[1] = _high_g_l;
  high_rgb[2] = _high_b_l;

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

  double cx, cy, fx_inv, fy_inv;
  double cx_c1, cy_c1, cz_c1;
  double cx_c2, cy_c2, cz_c2;
  float zd_c1;
  float zd_c2;
  points output;

  if( _use_depth )
    sleep(1);

  vector< vector<Point> > fullc;
  Mat dst, dst_range;

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

    original_center_e1.x = (ellipse_center.x + _off_x_l  );
    original_center_e1.y = (ellipse_center.y + _off_y_l  );

    if( _track_orientation ) {


      fullc.clear();
      fullc.push_back(outer_ellipse);
      Mat mask = Mat::zeros ( cropedImage.size(), cropedImage.type() );
      drawContours( mask, fullc, 0, Scalar(255,255,255), -1, 8 );
      cropedImage.copyTo(dst, mask );

      //imshow("dst", dst);
      //waitKey(1);

      inRange(dst, Scalar(0, 0, 254), Scalar(255, 231, 255), dst_range);
      Mat element = getStructuringElement( _dilation_elem, Size( 2*_dilation_size + 1, 2*_dilation_size+1 ), Point( _dilation_size, _dilation_size ));
      dilate( dst_range, dst_range, element );
      //imshow("cropedImage", dst_range);
      //waitKey(1);

      vector<vector<Point> > o_contours;
      findContours( dst_range, o_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      dst.release();
      mask.release();

      if( o_contours.size() == 0 ){
        cout << "[Warning!] Impossible to find orientation" << endl;
        original_center_e2.x = -1;
        original_center_e2.y = -1;
      }
      else {

        int max_c = -1000;
        int ellipse_candidate_c = -1;
        RotatedRect minEllipse;

        vector<Moments> mu(o_contours.size() );
        for(int cc=0; cc<o_contours.size(); cc++ ) {
          mu[cc] = moments( o_contours[cc], false );
          if( max_c < contourArea(o_contours[cc])) {
            max_c = contourArea(o_contours[cc]);
            ellipse_candidate_c = cc;
          } //Get better contour to describe our ellipse
        }

        if( ellipse_candidate_c != -1 ) {
          if( o_contours[ellipse_candidate_c].size() < 5 ) {
            cout << "[Warning!] less then 5 point in the found ellipse" << endl;
            original_center_e2.x = -1;
            original_center_e2.y = -1;
          }
          else {
            minEllipse = fitEllipse( Mat(o_contours[ellipse_candidate_c]) );
            circle( img, Point( minEllipse.center.x + _off_x_l, minEllipse.center.y + _off_y_l)  , 2, color, 2, 8 );

            original_center_e2.x = minEllipse.center.x + _off_x_l;
            original_center_e2.y = minEllipse.center.y + _off_y_l;
          }
        }
      }
    }

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
      if( _track_orientation )
        circle( img, original_center_e2, 2, color, 2, 8 );

      if( !img.empty())
        imshow( "img", img );
      waitKey(1);

    }

    if( _use_depth ) {
      cx = _cam1_cameraMatrix->at<double>(0,2);
      cy = _cam1_cameraMatrix->at<double>(1,2);
      fx_inv = 1.0 / _cam1_cameraMatrix->at<double>(0,0);
      fy_inv = 1.0 / _cam1_cameraMatrix->at<double>(1,1);
      zd_c1 = _depth_src.at<float>(original_center_e1.y,original_center_e1.x);

      cx_c1 = (zd_c1*0.001) * ( (original_center_e1.x - cx) * fx_inv );
      cy_c1 = (zd_c1*0.001) * ( (original_center_e1.y - cy) * fy_inv );
      cz_c1 = zd_c1*0.001;

      if( _track_orientation ) {
        if(original_center_e2.x != -1 && original_center_e2.y != -1 ) {
          zd_c2 = _depth_src.at<float>(original_center_e2.y,original_center_e2.x);
          cx_c2 = (zd_c2*0.001) * ( (original_center_e2.x - cx) * fx_inv );
          cy_c2 = (zd_c2*0.001) * ( (original_center_e2.y - cy) * fy_inv );
          cz_c2 = zd_c2*0.001;
        }
        else {
          cx_c2 = -1;
          cy_c2 = -1;
          cz_c2 = -1;
        }
      }
      else {
        cx_c2 = -1;
        cy_c2 = -1;
        cz_c2 = -1;
      }
    }
    else {
      //TODO img2space
    }

    output.p1.x = cx_c1;
    output.p1.y = cy_c1;
    output.p1.z = cz_c1;

    output.p2.x = cx_c2;
    output.p2.y = cy_c2;
    output.p2.z = cz_c2;

    //cout << "P0: [" << output.p1.x << " " << output.p1.y << " " << output.p1.z << "]" << endl;
    //cout << "P1: [" << output.p2.x << " " << output.p2.y << " " << output.p2.z << "]" << endl;
    //cout << endl;

    int w = write( _output_data_socket, &output, sizeof(output));
    //cout << "written: " << w << endl;

    r.sleep();
  }

}



void shape_tracking::save_images() {

  Mat img_l;
  Mat img_r;
	while( !_img_l_ready ) {
		usleep(0.1*1e6);
	}
  cout << "_img_l_ready: " << _img_l_ready << endl;

  while( !_img_r_ready ) {
		usleep(0.1*1e6);
	}
  cout << "_img_r_ready: " << _img_r_ready << endl;

  string line;
  cout << "Type enter to start image acquisition..." << endl;
  getline( cin, line );

  cout << "Acquiring 100 images" << endl;
  cout << "3..." << endl;
  sleep(1);
  cout << "2..." << endl;
  sleep(1);
  cout << "1..." << endl;
  sleep(1);

  int img_id = 0;

  std::string calib_dirpath_l = ros::package::getPath("shape_tracking") + "/calibration_raw_imgs/left";
  std::string calib_dirpath_r = ros::package::getPath("shape_tracking") + "/calibration_raw_imgs/right";

  while( img_id < 100 ) {
    img_l = _src_l;
    img_r = _src_r;

    cvtColor( img_l, img_l, CV_BGR2GRAY );
    cvtColor( img_r, img_r, CV_BGR2GRAY );



    Mat stereo_output;
    stereo_output.push_back( img_l );
    stereo_output.push_back( img_r );
    Size size(stereo_output.rows / 2, stereo_output.cols / 2);//the dst image size,e.g.100x100

    resize(stereo_output,stereo_output,size);//resize image

    imshow("output", stereo_output);
    waitKey(1);


    imwrite( calib_dirpath_l + "/left" + std::to_string(img_id) + ".png", img_l );
    imwrite( calib_dirpath_r + "/right" + std::to_string(img_id) + ".png", img_r );

    cout << (img_id+1) << " ... acquired!" << endl;
    img_id++;
    usleep(0.5*1e6);
  }
  cout << "Acquisition complete!" << endl;
  exit(1);


}


void shape_tracking::track_ellipses_stereo() {

  Mat img_l;
  Mat img_r;
	while( !_img_l_ready ) {
		usleep(0.1*1e6);
	}
  cout << "_img_l_ready: " << _img_l_ready << endl;

  while( !_img_r_ready ) {
		usleep(0.1*1e6);
	}
  cout << "_img_r_ready: " << _img_r_ready << endl;

  while( !_cam1_info_first ) {
		usleep(0.1*1e6);
	}

  while( !_cam2_info_first ) {
		usleep(0.1*1e6);
	}

  Scalar color = Scalar( 0, 0, 255 );

  ros::Rate r(_rate);

  int low_rgb_l[3]; int high_rgb_l[3];
  int low_rgb_r[3]; int high_rgb_r[3];
  low_rgb_l[0] = _low_r_l;
  low_rgb_l[1] = _low_g_l;
  low_rgb_l[2] = _low_b_l;
  high_rgb_l[0] = _high_r_l;
  high_rgb_l[1] = _high_g_l;
  high_rgb_l[2] = _high_b_l;

  low_rgb_r[0] = _low_r_r;
  low_rgb_r[1] = _low_g_r;
  low_rgb_r[2] = _low_b_r;
  high_rgb_r[0] = _high_r_r;
  high_rgb_r[1] = _high_g_r;
  high_rgb_r[2] = _high_b_r;

  vector<Point> outer_ellipse_l;
  vector<Point> inner_ellipse_l;
  Point ellipse_center_l;
  Point inner_ellipse_center_l;
  vector<vector<Point> > contours_l;
  int ellipse_min_c_l[2];
  int ellipse_max_c_l[2];
  ellipse_min_c_l[0] = ellipse_min_c_l[1] = 1000;
  ellipse_max_c_l[0] = ellipse_max_c_l[1] = -1000;
  geometry_msgs::Point p1_l, p2_l;

  vector<Point> outer_ellipse_r;
  vector<Point> inner_ellipse_r;
  Point ellipse_center_r;
  Point inner_ellipse_center_r;
  vector<vector<Point> > contours_r;
  int ellipse_min_c_r[2];
  int ellipse_max_c_r[2];
  ellipse_min_c_r[0] = ellipse_min_c_r[1] = 1000;
  ellipse_max_c_r[0] = ellipse_max_c_r[1] = -1000;
  geometry_msgs::Point p1_r, p2_r;
  bool to_dilate = true;

  bool invert_img = false;
  if( _set_th ) {
    namedWindow("Binary threshold", WINDOW_NORMAL);
    createTrackbar("bth","Binary threshold", &bin_th, 255, on_thresh_trackbar );
  }

  vector<Point> translated_c_l;
  Point original_center_e1_l;
  Point original_center_e2_l;

  vector<Point> translated_c_r;
  Point original_center_e1_r;
  Point original_center_e2_r;

  int p_camera[2];
  vector<double> p_space;

  double p_c1_space[2][3];
  double p_c2_space[2][3];

  cv::Mat pt_set1_pt_0 = cv::Mat(1,1,CV_32FC2);
  cv::Mat pt_set2_pt_0 = cv::Mat(1,1,CV_32FC2);
  cv::Mat pt_set1_pt_1 = cv::Mat(1,1,CV_32FC2);
  cv::Mat pt_set2_pt_1 = cv::Mat(1,1,CV_32FC2);
  Mat pt_3d_h_p0(1,1,CV_64FC4);
  Mat pt_3d_h_p1(1,1,CV_64FC4);

  double p03d[3];
  double p13d[3];
  points output;

  while(ros::ok()) {

    img_l = _src_l;
    img_r = _src_r;

    if( img_l.empty() || img_r.empty() )
      continue;

    ellipse_min_c_l[0] = ellipse_min_c_r[1] = 1000;
    ellipse_max_c_l[0] = ellipse_max_c_r[1] = -1000;

    outer_ellipse_l.clear();
    inner_ellipse_l.clear();
    contours_l.clear();

    //---Get first ellipse
    to_dilate = true;
    invert_img = false;

    //
    Mat cropedImage_l = img_l;
    if( _apply_roi )
      cropedImage_l = img_l(Rect( _off_x_l, _off_y_l, _rect_w_l, _rect_h_l));

    etrack->get_ellipse(cropedImage_l, _to_blur, low_rgb_l, high_rgb_l, invert_img, to_dilate, _dilation_elem, _dilation_size, false, false, outer_ellipse_l, ellipse_center_l);

    if( _track_orientation ) {
      for(int pts=0; pts<outer_ellipse_l.size(); pts++ ) {
        ellipse_min_c_l[0] = (ellipse_min_c_l[0] > outer_ellipse_l[pts].x ) ? outer_ellipse_l[pts].x : ellipse_min_c_l[0];
        ellipse_min_c_l[1] = (ellipse_min_c_l[1] > outer_ellipse_l[pts].y ) ? outer_ellipse_l[pts].y : ellipse_min_c_l[1];
        ellipse_max_c_l[0] = (ellipse_max_c_l[0] < outer_ellipse_l[pts].x ) ? outer_ellipse_l[pts].x : ellipse_max_c_l[0];
        ellipse_max_c_l[1] = (ellipse_max_c_l[1] < outer_ellipse_l[pts].y ) ? outer_ellipse_l[pts].y : ellipse_max_c_l[1];
      }
      Mat ellipse_roi_l = cropedImage_l(Rect( ellipse_min_c_l[0]-_roi_off_x, ellipse_min_c_l[1]-_roi_off_y, (ellipse_max_c_l[0]-ellipse_min_c_l[0])+_roi_off_x*2, (ellipse_max_c_l[1]-ellipse_min_c_l[1])+_roi_off_y*2) );
      Mat ellipse_roi_tmp_l;

      if( _set_th ) {
        _th = bin_th;
        cvtColor( ellipse_roi_l, ellipse_roi_tmp_l, CV_BGR2GRAY );
        threshold( ellipse_roi_tmp_l, ellipse_roi_tmp_l, _th, 255, 1 );
        imshow("Binary threshold",ellipse_roi_tmp_l);
        waitKey(1);
      }
      else {
        cvtColor( ellipse_roi_l, ellipse_roi_tmp_l, CV_BGR2GRAY );
        threshold( ellipse_roi_tmp_l, ellipse_roi_tmp_l, _th, 255, 1 );
        to_dilate = false;
        invert_img = true;
        etrack->get_ellipse(ellipse_roi_tmp_l, _to_blur, invert_img, to_dilate, _dilation_elem, _dilation_size, false, false, inner_ellipse_l, inner_ellipse_center_l);
      }
    }
    original_center_e1_l.x = (ellipse_center_l.x + _off_x_l  );
    original_center_e1_l.y = (ellipse_center_l.y + _off_y_l  );
    if( _track_orientation ) {
      original_center_e2_l.x = (inner_ellipse_center_l.x + _off_x_l + ellipse_min_c_l[0] );
      original_center_e2_l.y = (inner_ellipse_center_l.y + _off_y_l + ellipse_min_c_l[1] );
    }

    if( _show_img_elaboration ) {
      translated_c_l.clear();
      Point conts;
      for(int i=0; i<outer_ellipse_l.size(); i++) {
        conts.x = outer_ellipse_l[i].x + _off_x_l ;
        conts.y = outer_ellipse_l[i].y + _off_y_l;
        translated_c_l.push_back(conts);
      }
      contours_l.clear();
      contours_l.push_back( translated_c_l );
      drawContours( img_l, contours_l, 0, color, 2, 8 );

      circle( img_l, original_center_e1_l, 2, color, 2, 8 );
      if( _track_orientation )
        circle( img_l, original_center_e2_l, 2, color, 2, 8 );
    }

    outer_ellipse_r.clear();
    inner_ellipse_r.clear();
    contours_r.clear();

    //---Get first ellipse
    to_dilate = true;
    invert_img = false;


    Mat cropedImage_r = img_r;
    if( _apply_roi )
      cropedImage_r = img_r(Rect( _off_x_r, _off_y_r, _rect_w_r, _rect_h_r));

    etrack->get_ellipse(cropedImage_r, _to_blur, low_rgb_r, high_rgb_r, invert_img, to_dilate, _dilation_elem, _dilation_size, false, false, outer_ellipse_r, ellipse_center_r);

    if( _track_orientation ) {
      for(int pts=0; pts<outer_ellipse_r.size(); pts++ ) {
        ellipse_min_c_r[0] = (ellipse_min_c_r[0] > outer_ellipse_r[pts].x ) ? outer_ellipse_r[pts].x : ellipse_min_c_r[0];
        ellipse_min_c_r[1] = (ellipse_min_c_r[1] > outer_ellipse_r[pts].y ) ? outer_ellipse_r[pts].y : ellipse_min_c_r[1];
        ellipse_max_c_r[0] = (ellipse_max_c_r[0] < outer_ellipse_r[pts].x ) ? outer_ellipse_r[pts].x : ellipse_max_c_r[0];
        ellipse_max_c_r[1] = (ellipse_max_c_r[1] < outer_ellipse_r[pts].y ) ? outer_ellipse_r[pts].y : ellipse_max_c_r[1];
      }
      Mat ellipse_roi_r = cropedImage_r(Rect( ellipse_min_c_r[0]-_roi_off_x, ellipse_min_c_r[1]-_roi_off_y, (ellipse_max_c_r[0]-ellipse_min_c_r[0])+_roi_off_x*2, (ellipse_max_c_r[1]-ellipse_min_c_r[1])+_roi_off_y*2) );
      Mat ellipse_roi_tmp_r;

      if( _set_th ) {
        _th = bin_th;
        cvtColor( ellipse_roi_r, ellipse_roi_tmp_r, CV_BGR2GRAY );
        threshold( ellipse_roi_tmp_r, ellipse_roi_tmp_r, _th, 255, 1 );
        imshow("Binary threshold",ellipse_roi_tmp_r);
        waitKey(1);
      }
      else {
        cvtColor( ellipse_roi_r, ellipse_roi_tmp_r, CV_BGR2GRAY );
        threshold( ellipse_roi_tmp_r, ellipse_roi_tmp_r, _th, 255, 1 );
        to_dilate = false;
        invert_img = true;
        etrack->get_ellipse(ellipse_roi_tmp_r, _to_blur, invert_img, to_dilate, _dilation_elem, _dilation_size, false, false, inner_ellipse_r, inner_ellipse_center_r);
      }
    }

    original_center_e1_r.x = (ellipse_center_r.x + _off_x_r  );
    original_center_e1_r.y = (ellipse_center_r.y + _off_y_r  );

    if( _track_orientation ) {
      original_center_e2_r.x = (inner_ellipse_center_r.x + _off_x_r + ellipse_min_c_r[0] );
      original_center_e2_r.y = (inner_ellipse_center_r.y + _off_y_r + ellipse_min_c_r[1] );
    }
    if( _show_img_elaboration ) {
      translated_c_r.clear();
      Point conts;
      for(int i=0; i<outer_ellipse_r.size(); i++) {
        conts.x = outer_ellipse_r[i].x + _off_x_r ;
        conts.y = outer_ellipse_r[i].y + _off_y_r;
        translated_c_r.push_back(conts);
      }
      contours_r.clear();
      contours_r.push_back( translated_c_r );
      drawContours( img_r, contours_r, 0, color, 2, 8 );

      circle( img_r, original_center_e1_r, 2, color, 2, 8 );
      if( _track_orientation )
        circle( img_r, original_center_e2_r, 2, color, 2, 8 );
    }

    if( _show_img_elaboration ) {

      Mat stereo_output;
      stereo_output.push_back( img_l );
      stereo_output.push_back( img_r );

      Size size(stereo_output.rows / 2, stereo_output.cols / 2);//the dst image size,e.g.100x100
      resize(stereo_output,stereo_output,size);//resize image

      imshow("output", stereo_output);
      waitKey(1);
    }

    //Img2space conversion
    //--p0 camera 1 (left)
    p_camera[0] = original_center_e1_l.x;
    p_camera[1] = original_center_e1_l.y;
    img2space(p_camera, _cam1_cameraMatrix, _cam1_distCo, _cam1_R, _cam1_P, p_space );
    p_c1_space[0][0] = p_space[0];
    p_c1_space[0][1] = p_space[1];
    p_c1_space[0][2] = p_space[2];

    //--p1 camera 1
    if( _track_orientation ) {
      p_camera[0] = original_center_e2_l.x;
      p_camera[1] = original_center_e2_l.y;
      img2space(p_camera, _cam1_cameraMatrix, _cam1_distCo, _cam1_R, _cam1_P, p_space );
      p_c1_space[1][0] = p_space[0];
      p_c1_space[1][1] = p_space[1];
      p_c1_space[1][2] = p_space[2];
    }

    //--p1 camera 2 (right)
    p_camera[0] = original_center_e1_r.x;
    p_camera[1] = original_center_e1_r.y;
    img2space(p_camera, _cam2_cameraMatrix, _cam2_distCo, _cam2_R, _cam2_P, p_space );
    p_c2_space[0][0] = p_space[0];
    p_c2_space[0][1] = p_space[1];
    p_c2_space[0][2] = p_space[2];

    //--p1 camera 2
    if( _track_orientation ) {
      p_camera[0] = original_center_e2_r.x;
      p_camera[1] = original_center_e2_r.y;
      img2space(p_camera, _cam2_cameraMatrix, _cam2_distCo, _cam2_R, _cam2_P, p_space );
      p_c2_space[1][0] = p_space[0];
      p_c2_space[1][1] = p_space[1];
      p_c2_space[1][2] = p_space[2];
    }

    //---img2space conversion
    pt_set1_pt_0.at<cv::Vec2f>(0,0)[0] = p_c1_space[0][0]; //p0 c1 x
    pt_set1_pt_0.at<cv::Vec2f>(0,0)[1] = p_c1_space[0][1]; //p0 c1 y
    pt_set2_pt_0.at<cv::Vec2f>(0,0)[0] = p_c2_space[0][0]; //p1 c1 x
    pt_set2_pt_0.at<cv::Vec2f>(0,0)[1] = p_c2_space[0][1]; //p1 c1 y

    if( _track_orientation ) {
      pt_set1_pt_1.at<cv::Vec2f>(0,0)[0] = p_c1_space[1][0]; //p0 c2 x
      pt_set1_pt_1.at<cv::Vec2f>(0,0)[1] = p_c1_space[1][1]; //p0 c2 y
      pt_set2_pt_1.at<cv::Vec2f>(0,0)[0] = p_c2_space[1][0]; //p1 c2 x
      pt_set2_pt_1.at<cv::Vec2f>(0,0)[1] = p_c2_space[1][1]; //p1 c2 y
    }
    //---

    //---Stereo triangulation
    cv::triangulatePoints(P0, P10, pt_set1_pt_0, pt_set2_pt_0, pt_3d_h_p0);
    if( _track_orientation ) {
      cv::triangulatePoints(P0, P10, pt_set1_pt_1, pt_set2_pt_1, pt_3d_h_p1);
    }
    //---

    p03d[0] = pt_3d_h_p0.at<cv::Vec4f>(0,0)[0] / pt_3d_h_p0.at<cv::Vec4f>(0,0)[3];
    p03d[1] = pt_3d_h_p0.at<cv::Vec4f>(0,0)[1] / pt_3d_h_p0.at<cv::Vec4f>(0,0)[3];
    p03d[2] = pt_3d_h_p0.at<cv::Vec4f>(0,0)[2] / pt_3d_h_p0.at<cv::Vec4f>(0,0)[3];
    if( _track_orientation ) {
      p13d[0] = pt_3d_h_p1.at<cv::Vec4f>(0,0)[0] / pt_3d_h_p1.at<cv::Vec4f>(0,0)[3];
      p13d[1] = pt_3d_h_p1.at<cv::Vec4f>(0,0)[1] / pt_3d_h_p1.at<cv::Vec4f>(0,0)[3];
      p13d[2] = pt_3d_h_p1.at<cv::Vec4f>(0,0)[2] / pt_3d_h_p1.at<cv::Vec4f>(0,0)[3];
    }
    else {
      p13d[0] = p13d[1] = p13d[2] = -1;
    }

    output.p1.x = p03d[0];
    output.p1.y = p03d[1];
    output.p1.z = p03d[2];

    output.p2.x = p13d[0];
    output.p2.y = p13d[1];
    output.p2.z = p13d[2];

    int w = write( _output_data_socket, &output, sizeof(output));


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

  TooN::Vector<3> p0, p0cam1, p0cam2;
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
    boost::thread track_sphere_t( &shape_tracking::track_sphere, this );
  else if( _task == "ellipse_stereo_tracking")
    boost::thread track_ellipses_stero_t( &shape_tracking::track_ellipses_stereo, this );
  else if( _task == "save_images") {
    boost::thread track_ellipses_stero_t( &shape_tracking::save_images, this );
  }

  ros::spin();
}

int main( int argc, char** argv ) {

  ros::init( argc, argv, "shape_tracking");
  shape_tracking st;
  st.run();

  return 0;
} //main
