#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
//#include <opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
    {
//	CommandLineParser parser(argc, argv, "");
//		string cascadeName = parser.get<string>(1);
	   //CvCapture* capture = cvCaptureFromCAM(-1);
	   VideoCapture cap;
	   cap.open(0);
	   /*
	   if ( !cap ) {
		 fprintf( stderr, "ERROR: capture is NULL \n" );
		 getchar();
		 return -1;
	   }
	   * */
	   // Create a window in which the captured images will be presented
	   cvNamedWindow("mywindow", CV_WINDOW_FULLSCREEN );
	   cvNamedWindow("red", CV_WINDOW_FULLSCREEN );
	   // Show the image captured from the camera in the window and repeat
	   CascadeClassifier c;
			c.load("cascade.xml");
//	   c.load(cascadeName);
			Mat frame1;
			Mat frame2;
		   while ( 1 ) {
			 // Get one frame
			 //IplImage* frame = cvQueryFrame( capture );
			 /*
			 if ( !frame ) {
			   fprintf( stderr, "ERROR: frame is null...\n" );
			   getchar();
			   break;
			 }
			 * */
			 //Mat frame1 = cvarrToMat(frame);
			 cap >> frame2;

			 if( frame2.empty() )
                break;
             Mat gray;//, smallImg( cvRound (frame2.rows/1.1), cvRound(frame2.cols/1.1), CV_8UC1 );   
             //frame1.copyTo(frame2);   
//             frame2.copyTo(frame1);

				cvtColor( frame2, gray, COLOR_BGR2GRAY );
				//resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
			 
			 //equalizeHist( smallImg, smallImg);
			 //smallImg.copyTo(frame1);
			 //
			 equalizeHist( gray, gray);
			 //frame2.copyTo(gray);
			Mat lower_red_hue_range;
			Mat upper_red_hue_range;
			//inRange(gray, cv::Scalar(0, 0, 240), cv::Scalar(255, 255, 255), lower_red_hue_range);
			//inRange(gray, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
			inRange(gray, Scalar(0, 0, 204), Scalar(54, 53, 255), lower_red_hue_range);
			//inRange(gray, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
			//inRange(gray, cv::Scalar(17, 15, 100), cv::Scalar(50, 56, 200), upper_red_hue_range);
			//Mat red_hue_image;
			//addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
			//GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
			//Mat red_hue_image;
			//addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
			//GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
			//lower_red_hue_range.copyTo(frame1);
			Mat a;
			//bitwise_and(frame2, lower_red_hue_range, frame1, 0);
			cout <<"size 1121 = "<<lower_red_hue_range.size()<<endl;
				 std::vector<cv::Rect> ball;
				 
				 c.detectMultiScale(lower_red_hue_range, ball, 1.1 , 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
				 cout <<"size = "<<ball.size()<<endl;
				 for(std::vector<cv::Rect>::const_iterator r = ball.begin(); r < ball.end(); r++)
				 {
					 cv::Point center;
					 center.x = r->x + (int)r->width/2;
					 center.y = r->y + (int)r->height/2;
					 cv::ellipse(frame2, center, cv::Size(10, 10), 0, 0, 360, Scalar(0, 255, 0), 4, 8, 0);
				 }
				 
				 //show frames
				 imshow("red", lower_red_hue_range );
				 imshow("mywindow", frame2 );
			 // Do not release the frame!
			 //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
			 //remove higher bits using AND operator

			 if ( (cvWaitKey(10) & 255) == 27 ) break;
		   }
		   // Release the capture device housekeeping
		   //cvReleaseCapture( &capture );
		   //cvReleaseCapture(cap );
		   cvDestroyWindow( "mywindow" );
		   return 0;
    }
