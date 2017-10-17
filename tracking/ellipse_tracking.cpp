#include "ellipse_tracking.h"


ellipse_tracking::ellipse_tracking () {



}

void ellipse_tracking::get_ellipse(
  Mat img,
  bool blur_img, int low_rgb[3], int high_rgb[3], int dilation_elem, int dilation_size, bool disp_debug, bool disp_output,
  vector< Point> & ellips, Point & center ) {

  //---local init
  int max_c = -1000;
  int ellipse_candidate_c = -1;
  int ellipse_min_c[2];
  ellipse_min_c[0] = ellipse_min_c[1] = 1000;
  int ellipse_max_c[2];
  ellipse_max_c[0] = ellipse_max_c[1] = -1000;
  RotatedRect minEllipse;
  //---

  Mat img_th;
  vector<vector<Point> > contours;

  //Blur
  if( blur_img )
    GaussianBlur( img, img, Size(9, 9), 2, 2 );

  //Color based threshold
  inRange(img,Scalar(low_rgb[2], low_rgb[1], low_rgb[0]), Scalar(high_rgb[2], high_rgb[1], high_rgb[0]), img_th);

  if( disp_debug ) {
    imshow( "inRange", img_th );
    waitKey(1);
  }

  //Dilate
  Mat element = getStructuringElement( dilation_elem, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ));
  dilate( img_th, img_th, element );
  //---

  //Contours
  findContours( img_th, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  if( disp_debug ) {
    Mat drawing = Mat::zeros(img_th.size(), CV_8UC3 );

    for( int i = 0; i< contours.size(); i++ ) {
      Scalar color = Scalar( 0, 0, 255 );
      drawContours( drawing, contours, i, color, 2, 8 );
    }
    imshow( "contours", drawing );
    waitKey(1);
    drawing.release();
  }

  vector<Moments> mu(contours.size() );
  for(int cc=0; cc<contours.size(); cc++ ) {
    mu[cc] = moments( contours[cc], false );
    if( max_c < contourArea(contours[cc])) {
      max_c = contourArea(contours[cc]);
      ellipse_candidate_c = cc;
    } //Get better contour to describe our ellipse
  }

  if( ellipse_candidate_c != -1 ) {

    minEllipse = fitEllipse( Mat(contours[ellipse_candidate_c]) );
    center = minEllipse.center;
    ellips = contours[ellipse_candidate_c];

    if( disp_output ) {
      Scalar color = Scalar( 0, 0, 255 );
      ellipse( img, minEllipse, color, 2, 8 );
      circle( img, minEllipse.center, 2, color, 2, 8 );
      imshow("output", img);
      waitKey(1);

    }
  }

  img.release();
  img_th.release();
  element.release();
  contours.clear();

}
