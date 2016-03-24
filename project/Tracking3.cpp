#include "GotoGoal3.h"
#include <iostream>
#include <fstream>
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
#include <math.h>
#include <cmath>

//, X = 105f, px = 0.264583333333334f

//float z = (135.7648799 * 105)/(sizeImage * 0.264583333333334);
using namespace cv;
using namespace std;

const float f = 135.7648799, X = 202, px = 0.264583333333334;
const float maxWidth = 640, maxHeight = 480;
const float stopDistance = 250;
//#define X 105f
//#define px 0.264583333333334f

Tracking::Tracking(){
	delta = 80;
//	vmin = 162;
//	vmax = 256;
//	smin = 153;
	vmin = 77;
	vmax = 256;
	smin = 130;
//	trackBox = NULL;
};
Mat Tracking::readFrame(){
	cap >> frame;
	return frame;
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
		return 1;
	else
		return 0;
};
bool Tracking::detect(){
	 Mat image;
//	 cap >> frame;
	 frame.copyTo(image);
//	 frame.copyTo(image);
//	 namedWindow( "main", 0 );
	 cvtColor(image, hsv, COLOR_BGR2HSV);
	 int _vmin = vmin, _vmax = vmax;
	 cv::inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);
	 vector<Rect> ball;
	 c.detectMultiScale(image, ball, 1.1 , 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
//	 cout <<"size = "<<ball.size()<<endl;
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
	 imshow("main", image);
	 return true;
};
bool Tracking::trackObject(){
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    Rect trackWindow;
	Mat hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;


	int ch[] = {0, 0};
	hue.create(hsv.size(), hsv.depth());
	mixChannels(&hsv, 1, &hue, 1, ch, 1);
	Mat roi(hue, selection), maskroi(mask, selection);
	calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
	normalize(hist, hist, 0, 255, NORM_MINMAX);
	trackWindow = selection;
//	trackObject = 1;

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
	if (abs(width - height) >  delta || width < 10 || height < 10)
		return false;
	ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
	long  x = trackBox.center.x;

	cout << "x = " << x <<endl;
	return true;
};
void Tracking::trackbar(std::string nameWindow)
{
	namedWindow( nameWindow, 0 );
	createTrackbar( "Vmin", nameWindow, &vmin, 256, 0 );
	createTrackbar( "Vmax", nameWindow, &vmax, 256, 0 );
	createTrackbar( "Smin", nameWindow, &smin, 256, 0 );
}
void Tracking::showFrame()
{
	imshow( "threshold", mask );
//		imshow( "Histogram", histimg );
	imshow( "main", frame );
};
void Tracking::setFrame() {
	namedWindow( "main", 0 );
	namedWindow( "threshold", 0 );
	namedWindow( "Histogram", 0 );
}
void Tracking::inRange() {
//	Mat image;
	frame.copyTo(image);
	cvtColor(image, hsv, COLOR_BGR2HSV);
	int _vmin = vmin, _vmax = vmax;
	cv::inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);
//	ims
}
float Tracking::distance()
{
	float width = trackBox.size.width;
	float height = trackBox.size.height;
	float sizeImage = (width > height ? width : height);
	float z = (f * X)/(sizeImage * px);
	return z;
}
float Tracking::determindRotate() {
	long  x = trackBox.center.x;
	long  y = trackBox.center.y;
	cout << "x = " << x <<  "\ty = " << y <<endl;
	if (x <= 200)
		return (determindAngle(x, y));
//		return -20;
	else if (x <= 440)
		return 0;
	else
//		return 20;
		return (0 - determindAngle(x, y));
}
float Tracking::determindAngle(float x, float y) {
	float deltaX = y - maxHeight;
	float deltaY = maxWidth/2 - x;
	float cosAngle = abs(deltaX) /  sqrt(deltaX * deltaX + deltaY * deltaY);
	float angle = acos(cosAngle) * 180 / M_PI;
	return angle;
}
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
//	myRobot->setDirectMotionPrecedenceTime(1000);
//	myRobot->
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

/*
bool GotoGoal::disableAction(ArAction action){
	bool checked = false;
	return checked;
}; quoi khung
bool GotoGoal::enbleAction(ArAction action){
	bool checked = false;
	return checked;
};
*/
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
	//myRobot->setRotAccel()
	myRobot->setDeltaHeading(angle);
	myRobot->unlock();
//	ArLog::log(ArLog::Normal, "RotAccel = %f",myRobot->getRotAccel());
//	while(!myRobot->isHeadingDone());
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

int main(int argc, char **argv) {
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
		}
	}
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
//	namedWindow( "main", 0 );

	Tracking tracking;
//	tracking.setFrame();
	tracking.init();
	tracking.loadCascade();
//	tracking.trackbar("Trackbar");
	/*
	ArPose* poseList = readPostitions("positions.txt");
	int length = ARRAY_SIZE(poseList);
	for (int i = 0; i < length; i++) {
		gotoGoal.gotoGoal(poseList[i]);
		//ArLog(ArLog::Normal, "postive = %f, y = %f", poseList[i].getX(), poseList[i].getY());
		ArLog::log(ArLog::Normal, "postition x = %f, y = %f", poseList[i].getX(), poseList[i].getY());
		while (!gotoGoal.haveAchievedGoal()) {
			ArPose pose = gotoGoal.getPose();
			ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", pose.getX(), pose.getY());
			ArUtil::sleep(50);
		}

		int t = 1;
		gotoGoal.enableDirectionCommand();
		while(t * 90 <= 360) {
			gotoGoal.rotate(90);
			while(!gotoGoal.haveRotated());
			t++;
		}
		gotoGoal.disableDirectionCommand();
	}
	*/
	bool checkObject;
	gotoGoal.enableDirectionCommand();

	float angle = 0;

	namedWindow( "threshold", 0 );
	namedWindow( "Histogram", 0 );
	namedWindow( "main", 0 );
//	tracking.setFrame();
	float vel = 0;
	while(true) {
		Mat frame = tracking.readFrame();

		tracking.inRange();
		if (!checkObject)
			checkObject = tracking.detect();
		if (checkObject){
			if(tracking.trackObject()) {
				long distance = tracking.distance();
				if (distance <= 300) {
					gotoGoal.move(-200);
				} else if (distance <= 250){
					gotoGoal.stop();
				} else {
					vel = distance * 0.7;
					vel = (int) (vel/50) * 50;
					if (vel > 200)
						vel = 200;
					gotoGoal.setVel(vel);
				}
				angle =  tracking.determindRotate();
				cout <<"khoang cach: "<<distance<<"\tGoc quay: "<<angle<<"\t van toc = "<<vel<<endl;
				if (angle != 0) {
					gotoGoal.stop();
					gotoGoal.rotate(angle);
				}

			} else {
				checkObject = false;
				cout<< "Bat sai"<<endl;
			}
		} else {
			gotoGoal.stop();
			cout<< "Bat lai doi tuong"<<endl;
		}
//		tracking.showFrame();
//		imshow("main", frame);
	}
	gotoGoal.shutdown();
}
