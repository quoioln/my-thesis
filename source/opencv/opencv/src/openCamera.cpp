#include  <cstdlib>
#include <cstdio>
#include <iostream>
#include <stdlib.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace std;

int main(int argc, char** argv)
{

	CvCapture* capture = cvCaptureFromCAM(-1);
	//IplImage* image = cvQueryFrame(capture);
//	cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
	//cv::imshow( "mywindow", image );
	//cvSaveImage()
	char * fileName;
	fileName = new char[50];
	strcpy(fileName, "positive_");

	int i = 0;
	while(true) {

		char* t = new char[5];
		sprintf (t, "%d", i);

		strcpy(fileName,t);
		strcpy(fileName,".jpg");
		printf("name %s", fileName);
		//itoa (i,t,10);
		//sprintf(t, "%d", i);
//		str
		//cout<<"number = "<<t;
		 IplImage* frame = cvQueryFrame( capture );
		 cvShowImage( "mywindow", frame );
		         // Do not release the frame!
		         //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
		         //remove higher bits using AND operator

//		 snprintf(t, 5, "%d", i);

		if ( cvWaitKey(10)  == 27 ) {
			i ++;


			//cout<<""<<fileName;
			cvSaveImage(fileName, frame);
		}

	}

	//image->
   // Release the capture device housekeeping
   cvReleaseCapture( &capture );
   cvDestroyWindow( "mywindow" );
   return 0;
}
