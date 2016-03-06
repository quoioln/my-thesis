#include "Tracking.h"
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/core/types_c.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Tracking::Tracking(){
	delta = 20f;
	vmin = 10;
	vmax = 256;
	smin = 30;
	trackBox = NULL;
};

int Tracking::init(){
	cap.open(0);
	if(!cap.isOpened()){
		cout << "***Could not initialize capturing***";
		return 0;
	}
	return 1;
};
int Tracking::loadCascade(){
	if(c.load("cascade.xml"))
		return 0;
	else
		return 1;
};
bool Tracking::detect(){
	 Mat image;
	 cap >> frame;
	 frame.copyTo(image);
	 vector<Rect> ball;
	 c.detectMultiScale(image, ball, 1.1 , 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
	 cout <<"size = "<<ball.size()<<endl;
	 int sizeball = ball.size();
	 if (sizeball != 1)
		 return false;
	 vector<Rect>::const_iterator r = ball.begin();
	 if (abs((float)r->width - (float)r->height) > delta)
		 return false;
	 selection.x = r->x;
	 selection.y = r->y;
	 selection.width = r->x + (int)r->width;
	 selection.height = r->y + (int)r->height;
	 selection &= Rect(0, 0, image.cols, image.rows);
	 return true;
};
bool Tracking::trackObject(){
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    Rect trackWindow;
	Mat hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	Mat image;
	frame.copyTo(image);
	cvtColor(image, hsv, COLOR_BGR2HSV);
	int _vmin = vmin, _vmax = vmax;
	inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);
	int ch[] = {0, 0};
	hue.create(hsv.size(), hsv.depth());
	mixChannels(&hsv, 1, &hue, 1, ch, 1);
	Mat roi(hue, selection), maskroi(mask, selection);
	calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
	normalize(hist, hist, 0, 255, NORM_MINMAX);
	trackWindow = selection;
	trackObject = 1;

	histimg = Scalar::all(0);
	int binW = histimg.cols / hsize;
	Mat buf(1, hsize, CV_8UC3);
	for( int i = 0; i < hsize; i++ )
		buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
	cvtColor(buf, buf, COLOR_HSV2BGR);

	for( int i = 0; i < hsize; i++ )
	{
		int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
		rectangle( histimg, Point(i*binW,histimg.rows),
				   Point((i+1)*binW,histimg.rows - val),
				   Scalar(buf.at<Vec3b>(i)), -1, 8 );
	}
	calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
	backproj &= mask;
	trackBox = CamShift(backproj, trackWindow,
	TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
	float width = trackBox.size.width;
	float height = trackBox.size.height;
	if (abs(width - height) >  delta || width < 20 || height < 20)
		return false;
	ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
	imshow( "threshold", mask );
	imshow( "Histogram", histimg );
	imshow( "main", image );
	return true;
};
void Tracking::trackbar(string nameWindow)
{
	namedWindow( nameWindow, 0 );
	createTrackbar( "Vmin", nameWindow, &vmin, 256, 0 );
	createTrackbar( "Vmax", nameWindow, &vmax, 256, 0 );
	createTrackbar( "Smin", nameWindow, &smin, 256, 0 );
}
void Tracking::showFrame(string nameWindow, Mat frame)
{
	imshow( nameWindow, frame );
};
float Tracking::distance()
{
	float width = trackBox.size.width;
	float height = trackBox.size.height;
	float sizeImage = (width > height ? width : height);
	float z = (513 * 0.0264583333333334)/sizeImage;
	return z;
}
float Tracking::determindRotate() {
//	float width = trackBox.size.width;
//	float height = trackBox.size.height;
	float x = trackBox.center.x;
	if (x <= 140)
		return 60;
	else if (x <= 280)
		return 30;
	else if (x <= 360)
		return 0;
	else if (x <= 500)
		return -30;
	else
		return -60;
}
