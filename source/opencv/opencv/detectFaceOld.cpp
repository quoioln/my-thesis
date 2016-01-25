#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay( Mat frame );

/** Global variables */
String face_cascade_name = "cascades.xml";
//String eyes_cascade_name = "haarcascades/haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
//CascadeClassifier eyes_cascade;
String window_name = "Capture - Face detection";

/** @function main */
int main( void )
{
//    VideoCapture capture;
//	cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
    CvCapture* capture = cvCaptureFromCAM(-1);
    cvNamedWindow( "window_name", CV_WINDOW_AUTOSIZE );
    Mat frame;
    //-- 1. Load the cascades
    if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade\n"); return -1; };
//    if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading eyes cascade\n"); return -1; };

    //-- 2. Read the video stream
    //capture.open( -1 );
    //if ( ! capture-> ) { printf("--(!)Error opening video capture\n"); return -1; }
    CvMemStorage* storage = 0;
    storage = cvCreateMemStorage(0);
    while (true )
    {
    	IplImage* img = cvQueryFrame( capture );
    	frame = cvarrToMat(img);

    	if( frame.empty() )
        {
            printf(" --(!) No captured frame -- Break!");
            break;
        }
    	CvSeq* twarze = cvHaarDetectObjects(img, face_cascade, storage,
                1.1, 3, CV_HAAR_DO_CANNY_PRUNING);
        //-- 3. Apply the classifier to the frame
        detectAndDisplay( frame, twarze );

        int c = waitKey(30);
        if( (char)c == 27 ) { break; } // escape
    }
    return 0;
}

/** @function detectAndDisplay */
void detectAndDisplay( Mat frame, CvSeq*  twarze)
{
	/*
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

//        Mat faceROI = frame_gray( faces[i] );
//        std::vector<Rect> eyes;

        //-- In each face, detect eyes
   /*
         eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(30, 30) );

        for ( size_t j = 0; j < eyes.size(); j++ )
        {
            Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            circle( frame, eye_center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
        }
        */
    //}

for(int i = 0; i < twarze->total; i++ )
{

    CvRect* r = (CvRect*)cvGetSeqElem( twarze, i );

    pt1.x = r->x*scale;
    pt2.x = (r->x+r->width)*scale;
    pt1.y = r->y*scale;
    pt2.y = (r->y+r->height)*scale;


    cvRectangle( img, pt1, pt2, CV_RGB(100,250,50), 3, 8, 0 );
}
    //-- Show what you got
    imshow( "window_name", frame );
    cvWaitKey(30);
}