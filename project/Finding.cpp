#include <iostream>
#include <fstream>
#include "Aria.h"
#include "Arnl.h"
#include "ArSonarLocalizationTask.h"
#include "ArSystemStatus.h"
#include "GotoGoal3.h"
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
#include <math.h>
#include <cmath>
#define ARRAY_SIZE(array)(sizeof(array[0])/sizeof(array))

using namespace cv;
using namespace std;

const float f = 135.7648799, X = 202, px = 0.264583333333334;
const float maxWidth = 640, maxHeight = 480, delta = 40;
const float stopDistance = 250;
Mat image;
Rect selection;
RotatedRect trackBox;
//RotatedRect trackBox;
//#define X 105f
//#define px 0.264583333333334f
int vmin = 77;
int	vmax = 256;
int	smin = 130;

GotoGoal::GotoGoal(ArRobot* robot, ArSonarDevice* sonar, ArServerBase* server, ArServerInfoRobot* serverInfo){
	this->myRobot = robot;
	this->sonarDev = sonar;
	this->server = server;
	this->serverInfo = serverInfo;

//	this->serverInfo = ArServerInfoRobot(this->server, this->myRobot);
}
void GotoGoal::init(int argc, char **argv){
	myRobot->runAsync(true);

	myRobot->moveTo(ArPose(0,0,0));
	myRobot->comInt(ArCommands::ENABLE, 1);
	myRobot->addRangeDevice(sonarDev);
	gotoGoalAction = ArActionGoto("goto", ArPose(0, 0, 0), 200);
	avoidFrontAction = ArActionAvoidFront("avoid front", 400, 200, 10);
	myRobot->addAction(&gotoGoalAction, 50);
	myRobot->addAction(&avoidFrontAction, 60);
	myRobot->setStateReflectionRefreshTime(100);
	server->runAsync();
	myRobot->enableMotors();
	myRobot->lock();
	myRobot->setRotAccel(5000);

	myRobot->unlock();

};
void GotoGoal::stop(){
	myRobot->lock();
	myRobot->stop();
	myRobot->setVel(0);
	myRobot->unlock();
};

void GotoGoal::gotoGoal(ArPose pose){

	if (!gotoGoalAction.isActive()) {
		ArLog::log(ArLog::Normal, "action goto goal is deactive");
		return;
	}
	if (!avoidFrontAction.isActive()) {
		ArLog::log(ArLog::Normal, "action avoid front is deactive. Robot is unsafe");
		return;
	}

	gotoGoalAction.setGoal(pose);
};
void GotoGoal::rotate(float angle){

	myRobot->lock();
	myRobot->setDeltaHeading(angle);
	myRobot->unlock();
};
void GotoGoal::setVel(float vel){
	myRobot->lock();
	myRobot->setVel(vel);
	myRobot->unlock();
};
bool GotoGoal::haveAchievedGoal(){
	return gotoGoalAction.haveAchievedGoal();
};
bool GotoGoal::haveRotated(){
	return myRobot->isHeadingDone();
};
void GotoGoal::enableDirectionCommand(){
	gotoGoalAction.deactivate();
	avoidFrontAction.deactivate();
};
void GotoGoal::disableDirectionCommand(){
	myRobot->clearDirectMotion();
	gotoGoalAction.activate();
	avoidFrontAction.activate();

};
ArPose GotoGoal::getPose(){
	return myRobot->getPose();
}
void GotoGoal::shutdown(){
	Aria::shutdown();
};
void GotoGoal::lock(){
	myRobot->lock();
};
void GotoGoal::unlock(){
	myRobot->unlock();
};
void GotoGoal::move(int distance) {
	myRobot->lock();
	myRobot->move(distance);
	myRobot->unlock();
}
bool detect(Mat frame, CascadeClassifier cascade) {
	std::vector<cv::Rect> ball;
	cascadex.detectMultiScale(frame, ball, 1.1 , 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
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
}
bool trackObject(Mat hsv, Mat mask){
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    Rect trackWindow;
	Mat hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;

	int ch[] = {0, 0};
	Mat hue;
	hue.create(hsv.size(), hsv.depth());
	mixChannels(&hsv, 1, &hue, 1, ch, 1);
	Mat roi(hue, selection), maskroi(mask, selection);
	calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
	normalize(hist, hist, 0, 255, NORM_MINMAX);
	trackWindow = selection;

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
	if (abs(width - height) >  delta)// || width < 10 || height < 10)
		return false;
	ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
	long  x = trackBox.center.x;

	cout << "x = " << x <<endl;
	return true;
};
float distance()
{
	float width = trackBox.size.width;
	float height = trackBox.size.height;
	float sizeImage = (width > height ? width : height);
	float z = (f * X)/(sizeImage * px);
	return z;
}
float determindAngle(float x, float y) {
	float deltaX = y - maxHeight;
	float deltaY = maxWidth/2 - x;
	float cosAngle = abs(deltaX) /  sqrt(deltaX * deltaX + deltaY * deltaY);
	float angle = acos(cosAngle) * 180 / M_PI;
	return angle;
}
float determindRotate() {
	long  x = trackBox.center.x;
	long  y = trackBox.center.y;
	cout << "x = " << x <<  "\ty = " << y <<endl;
	if (x <= 200)
		return (0 - determindAngle(x, y));
	else if (x <= 440)
		return 0;
	else
		return determindAngle(x, y);
}

void addGoalDone(ArPose pose) {
	ArLog::log(ArLog::Normal, "x = %f, y = %f", pose.getX(), pose.getY());
}
ArPose* readPostitions(char* fileName){
	ArPose* postitionList = new ArPose[1000];
	ArPose pose;
	ifstream is(fileName);
	char line[20];
	bool check = true;
	int i = 0;
	while (!is.eof()) {
		is >>line;
		cout <<"*"<<atoi(line)<<"*"<<endl;
		if (check) {

			pose.setX(atoi(line));
			check = false;
		} else {
			pose.setY(atoi(line));
			postitionList[i] = pose;
			check = true;
			i++;
		}
	}
	is.close();
	return postitionList;
}
int main (int argc, char** argv) {
	Aria::init();
	ArRobot robot;
	ArSonarDevice sonar;
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	if (!robotConnector.connectRobot()) {
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if(parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}

	ArSonarDevice sonarDev;
	ArPose* poseList = readPostitions("positions.txt");
	robot.runAsync(true);
	robot.enableMotors();
	robot.moveTo(ArPose(0,0,0));
	robot.comInt(ArCommands::ENABLE, 1);
	robot.addRangeDevice(&sonarDev);
	ArActionGoto gotoPoseAction("goto", ArPose(0, 0, 0), 200);
	ArActionAvoidFront avoidFront("avoid front");
	ArActionStallRecover stallRecover("stall recover");
	robot.addAction(&gotoPoseAction, 50);
	robot.addAction(&avoidFront, 60);
	robot.moveTo(ArPose(0,0,0));
	int length = ARRAY_SIZE(poseList);
	cout<<"do dai"<<length;
	ArServerBase server;
	ArServerSimpleOpener simpleOpener(&parser);
	char fileDir[1024];
	  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(),
				 "ArNetworking/examples");

	  // first open the server up
	  if (!simpleOpener.open(&server, fileDir, 240))
	  {
	    if (simpleOpener.wasUserFileBad())
	      printf("Bad user/password/permissions file\n");
	    else
	      printf("Could not open server port\n");
	    exit(1);
	  }
	ArServerInfoRobot serverInfo(&server, &robot);
	GotoGoal gotoGoal(&robot, &sonar, &server, &serverInfo);
	gotoGoal.init(argc, argv);
	float angle = 0;
	VideoCapture cap;
	cap.open(0);
	Rect trackWindow;
	//var check find ball
	bool checkObject = false;
	int hsize = 16;

	namedWindow( "threshold", 0 );
	namedWindow( "trackbar", 0 );
	namedWindow( "Histogram", 0 );
	namedWindow( "main", 0 );
	createTrackbar( "Vmin", "trackbar", &vmin, 256, 0 );
	createTrackbar( "Vmax", "trackbar", &vmax, 256, 0 );
	createTrackbar( "Smin", "trackbar", &smin, 256, 0 );

	CascadeClassifier c;
	c.load("cascade.xml");
	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	float vel = 0;
	int i = 0;
	while(1)
	{
		cap >> frame;
		if( frame.empty() ){
			cout<<"error camera"<<endl;
			break;
		}
		frame.copyTo(image);
		cvtColor(image, hsv, COLOR_BGR2HSV);
		int _vmin = vmin, _vmax = vmax;
		inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),	Scalar(180, 256, MAX(_vmin, _vmax)), mask);
		gotoPoseAction.setGoal(poseList[i]);
		while (!gotoPoseAction.haveAchievedGoal()) 
		{
			ArLog::log(ArLog::Normal, "goal(%.2f, %0.2f) x = %.2f, y = %.2f", poseList[i].getX(), poseList[i].getY(), robot.getX(), robot.getY());
//			if (!checkObject)
			   checkObject = detect(frame, c);
			if (checkObject)
				cout <<"Phat hien doi tuong"<<endl;
			else
				cout <<"Khong phat hien doi tuong"<<endl;
			if (checkObject) {
				if(trackObject(hsv, mask)) {
					float d = distance();
					if (d < 250) {
						gotoGoal.move(-200);
					} else if ( d >= 250 && d <= 300) {
						gotoGoal.stop();
					}
					else {
						vel = d * 0.7;
						vel = (int) (vel/50) * 50;
						if(vel > 200) {
							vel = 200;
							gotoGoal.setVel(vel);
						}
						angle =  determindRotate();
						cout <<"khoang cach: "<<d<<"\tGoc quay: "<<angle<<"\t van toc = "<<vel<<endl;
						if (angle != 0) {
							gotoGoal.stop();
							gotoGoal.rotate(angle);
						}
					}
				}
			}
			imshow("main", image);
			imshow( "threshold", mask );
			imshow( "Histogram", histimg );
		}
		i++;
	}

	ArUtil::sleep(2000);
	Aria::shutdown();

}
