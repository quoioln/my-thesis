#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
    {

		   CvCapture* capture = cvCaptureFromCAM(-1);
		   if ( !capture ) {
			 fprintf( stderr, "ERROR: capture is NULL \n" );
			 getchar();
			 return -1;
		   }
		   // Create a window in which the captured images will be presented
		   cvNamedWindow("mywindow", CV_WINDOW_FULLSCREEN );
		   // Show the image captured from the camera in the window and repeat
		   while ( 1 ) {
			 // Get one frame
			 IplImage* frame = cvQueryFrame( capture );
			 if ( !frame ) {
			   fprintf( stderr, "ERROR: frame is null...\n" );
			   getchar();
			   break;
			 }
			 Mat frame1 = cvarrToMat(frame);
				 CascadeClassifier c;
				 c.load("cascade.xml");
				 std::vector<cv::Rect> ball;
				 c.detectMultiScale(frame1, ball, 1.1 , 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
				 for(std::vector<cv::Rect>::const_iterator r = ball.begin(); r < ball.end(); r++)
				 {
					 cv::Point center;
					 center.x = r->x + (int)r->width/2;
					 center.y = r->y + (int)r->height/2;
					 cv::ellipse(frame1, center, cv::Size(r->width/2, r->height/2), 0, 0, 360, Scalar(0, 255, 0), 4, 8, 0);
				 }
				 //show frames
				 imshow("mywindow", frame1 );
			 // Do not release the frame!
			 //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
			 //remove higher bits using AND operator

			 if ( (cvWaitKey(10) & 255) == 27 ) break;
		   }
		   // Release the capture device housekeeping
		   cvReleaseCapture( &capture );
		   cvDestroyWindow( "mywindow" );
		   return 0;
    }
