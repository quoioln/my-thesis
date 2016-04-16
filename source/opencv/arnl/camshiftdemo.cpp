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

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
//Rect selection;
Rect selection;

//int vmin = 10, vmax = 256, smin = 30;
//int vmin = 162, vmax = 256, smin = 153;
int vmin = 77, vmax = 256, smin = 130;
static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        cout << "left mouse down"<<endl;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1;
        cout << "left mouse  up"<<endl;
        break;
    }
}

string hot_keys =
    "\n\nHot keys: \n"
    "\tESC - quit the program\n"
    "\tc - stop the tracking\n"
    "\tb - switch to/from backprojection view\n"
    "\th - show/hide object histogram\n"
    "\tp - pause video\n"
    "To initialize tracking, select the object with mouse\n";

static void help()
{
    cout << "\nThis is a demo that shows mean-shift based tracking\n"
            "You select a color objects such as your face and it tracks it.\n"
            "This reads from video camera (0 by default, or the camera number the user enters\n"
            "Usage: \n"
            "   ./camshiftdemo [camera number]\n";
    cout << hot_keys;
}

const char* keys =
{
    "{help h | | show help message}{@camera_number| 0 | camera number}"
};

int main( int argc, const char** argv )
{
    VideoCapture cap;
    Rect trackWindow;
    //var check find ball
    bool checkObject = false;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    int camNum = parser.get<int>(0);
    cap.open(camNum);

    if( !cap.isOpened() )
    {
        help();
        cout << "***Could not initialize capturing...***\n";
        cout << "Current parameter's value: \n";
        parser.printMessage();
        return -1;
    }
    cout << hot_keys;
    namedWindow( "Histogram", 0 );
    namedWindow( "CamShift Demo", 0 );
    createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
    createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
    createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );

    CascadeClassifier c;
	c.load("cascade.xml");

    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false;
    for(;;)
    {
        if( !paused )
        {
            cap >> frame;
            if( frame.empty() )
                break;
        }
//        image = fastNlMeansDenoising(image, image);
//        cvThreshold(frame, frame, 70, 255, CV_THRESH_BINARY);
        frame.copyTo(image);
        frame.copyTo(mask);
//        cvtColor(image, hsv, COLOR_BGR2HSV);
//        inRange(hsv, Scalar(0, smin, MIN(vmin,vmax)),
//                                Scalar(180, 256, MAX(vmin, vmax)), mask);
//        frame.copyTo(mask);


			 if (!checkObject) {
				 std::vector<cv::Rect> ball;
				 c.detectMultiScale(frame, ball, 1.1 , 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
				 cout <<"size = "<<ball.size()<<endl;
				 if (ball.size() == 1){
//				 for(std::vector<cv::Rect>::const_iterator r = ball.begin(); r < ball.end(); r++)
//				 {
					 Rect r = ball.front();
					 cv::Point center;
					 center.x = r.x + (int)r.width/2;
					 center.y = r.y + (int)r.height/2;
					 cv::ellipse(frame, center, cv::Size(r.width/2, r.height/2), 0, 0, 360, Scalar(0, 255, 0), 4, 8, 0);
					 selection.x = r.x;
					 selection.y = r.y;
					 selection.width = r.x + (int)r.width;
					 selection.height = r.y + (int)r.height;
	//				 selection.y = MIN(y, origin.y);
	//				 selection.width = std::abs(x - origin.x);
	//				 selection.height = std::abs(y - origin.y);

					 selection &= Rect(0, 0, image.cols, image.rows);
	//				 selection = r;
					 checkObject = true;
				 }
//				 if (ball.size() > 0)
//					 checkObject = true;

			 } else {

//			 while (1){

				cvtColor(image, hsv, COLOR_BGR2HSV);

//				if( trackObject )
//				{
					int _vmin = vmin, _vmax = vmax;

					inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
							Scalar(180, 256, MAX(_vmin, _vmax)), mask);
					int ch[] = {0, 0};
					hue.create(hsv.size(), hsv.depth());
					mixChannels(&hsv, 1, &hue, 1, ch, 1);

//					if( trackObject < 0 )
//					{
						Mat roi(hue, selection), maskroi(mask, selection);
//						Mat roi()
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
//					}

					calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
					backproj &= mask;
					RotatedRect trackBox = CamShift(backproj, trackWindow,
										TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));

					if( trackWindow.area() <= 1 )
					{
						int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
						trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
										   trackWindow.x + r, trackWindow.y + r) &
									  Rect(0, 0, cols, rows);
					}

					if( backprojMode )
						cvtColor( backproj, image, COLOR_GRAY2BGR );
//					trackWindow.
//					trackBox.
//					float width = trackWindow.size().width;
//					float height = trackWindow.size().height;
					long  x = trackBox.center.x;

					cout << "x = " << x <<endl;

					float width = trackBox.size.width;
					float height = trackBox.size.height;

				if (abs(width - height) <= 80 && width > 20 && height > 20) {
//					if (image.size > 0)
						ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
//					else

//				}
//				if( selectObject && selection.width > 0 && selection.height > 0 )
//				{
//					Mat roi(image, selection);
//					bitwise_not(roi, roi);
//				}
//				if (trackObject >=0)
					imshow( "threshold", mask );
					imshow( "Histogram", histimg );
				} else {
					checkObject = false;
				}
			}
//			 }


        imshow( "CamShift Demo", image );


        char c = (char)waitKey(10);
        if( c == 27 )
            break;
    }

    return 0;
}
