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

class Tracking{
private:
//	bool trackOject;
	float delta;
	Rect selection;
	VideoCapture cap;
	Mat frame;
	CascadeClassifier c;
	int vmin;
	int vmax;
	int smin;
	RotatedRect trackBox;
	Mat hue, mask, image, hsv;
public:
	Tracking();
	Mat readFrame();
	int init();
	int loadCascade();
	bool detect();
	bool trackObject();
	void trackbar(std::string nameWindow);
	void setFrame();
	void showFrame();
	void inRange();
	float distance();
	float determindRotate();
	float determindAngle(float x, float y);
};
