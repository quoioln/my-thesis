#include "GotoGoal3.h"
#include <iostream>
#include <fstream>
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
#include "sstream"
using namespace cv;
using namespace std;

const float f = 135.7648799, X = 100, px = 0.264583333333334;
const float maxWidth = 640, maxHeight = 480, delta = 40;
const float stopDistance = 250;
const int lenght = 25;
const int timeOut = 180000;
const int myAngle = 20;

Mat image, mask, hsv, frame;
Rect selection;
RotatedRect trackBox;
int vmin = 77;
int	vmax = 256;
int	smin = 130;

//var check find ball
bool checkObject = false;
bool check = false;

int findObject = 0;
bool test;

GotoGoal gotoGoal;
void activeMotor(ArNetPacket * packet)
{
	gotoGoal.enableRobot();
	gotoGoal.disableDirectionCommand();
}
void deactiveRobot(ArNetPacket * packet)
{
	gotoGoal.disableRobot();
//	gotoGoal.disableDirectionCommand();
}
void sendData(ArServerClient* client, ArNetPacket*) {
	ArNetPacket reply;
	reply.doubleToBuf(findObject);
    client->sendPacketUdp(&reply);
}

void sendPoseRobot(ArServerClient* client, ArNetPacket*) {
	ArNetPacket reply;
	ArPose pose = gotoGoal.getPose();
	reply.doubleToBuf(pose.getX());
	reply.doubleToBuf(pose.getY());
//	stringstream content;
//	content<<"x = "<<pose.getX()<<"  y = "<<pose.getY();
//	cout <<"x = "<<pose.getX()<<"\ty = "<<pose.getY()<<endl;
    client->sendPacketUdp(&reply);
}

GotoGoal::GotoGoal(){
	this->myRobot = NULL;
	this->sonarDev = NULL;
	this->server = NULL;
	this->serverInfo = NULL;
	this->serverHanlerCommands = NULL;
	this->serverSimpleComUC = NULL;
	this->serverFileToClient = NULL;
}
GotoGoal::GotoGoal(ArRobot* robot, ArSonarDevice* sonar, ArServerBase* server, ArServerFileToClient * serverFileToClient){
	this->myRobot = robot;
	this->sonarDev = sonar;
	this->server = server;
	this->serverInfo = new ArServerInfoRobot(this->server, this->myRobot);
	this->serverHanlerCommands = new ArServerHandlerCommands(this->server);
	this->serverSimpleComUC = new ArServerSimpleComUC(this->serverHanlerCommands, this->myRobot);
	this->serverFileToClient = serverFileToClient;//ArServerFileToClient(this->server, ".");
	ArNetPacket packet;
	this->serverHanlerCommands->addCommand("requestEnableMotor", "request enable motor", new ArGlobalFunctor1<ArNetPacket *>(&activeMotor));
	this->serverHanlerCommands->addCommand("requestDisableMotor", "request disable motor", new ArGlobalFunctor1<ArNetPacket *>(&deactiveRobot));
//	this->serverHanlerCommands->addCommand("test", "test", new ArGlobalFunctor2<ArServerClient*, ArNetPacket*>(&test));
	this->server->addData("handleCheckObjectData", "some wierd test", new ArGlobalFunctor2<ArServerClient*, ArNetPacket*>(&sendData), "none", "none");
	this->server->addData("handlePoseRobot", "pose robot", new ArGlobalFunctor2<ArServerClient*, ArNetPacket*>(&sendPoseRobot), "none", "none");
	this->server->broadcastPacketTcp(&packet, "handleCheckObjectData");
	this->server->broadcastPacketTcp(&packet, "handlePoseRobot");
}
void GotoGoal::init(int argc, char **argv){
	myRobot->runAsync(true);
//	myRobot->comInt(ArCommands::ENABLE, 1);
	myRobot->addRangeDevice(sonarDev);
	gotoGoalAction = ArActionGoto("goto", ArPose(0, 0, 0), 1000, 200, 100, 20);
	avoidFrontAction = ArActionAvoidFront("avoid front", 600, 200, 20, true);
	stallRecover = ArActionStallRecover("stall recover", 400, 50, 200, 20, true);
//	avoidSide = ArActionAvoidSide("", 200, 20);
	myRobot->addAction(&gotoGoalAction, 50);
	myRobot->addAction(&avoidFrontAction, 60);
//	myRobot->addAction(&avoidSide, 50);
	myRobot->addAction(&stallRecover, 70);

	server->runAsync();
//	myRobot->enableMotors();
	myRobot->lock();
//	myRobot->setRotAccel(2000);
//	myRobot->setVel(200);
	myRobot->unlock();
};
void GotoGoal::stop(){
	myRobot->lock();
	myRobot->stop();
	myRobot->setVel(0);
	myRobot->unlock();
};
void GotoGoal::enableRobot() {
	myRobot->enableMotors();
}
void GotoGoal::disableRobot(){
	myRobot->disableMotors();
}
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
	stallRecover.deactivate();
};
void GotoGoal::disableDirectionCommand(){
	myRobot->clearDirectMotion();
	gotoGoalAction.activate();
	avoidFrontAction.activate();
	stallRecover.activate();

};
ArPose GotoGoal::getPose(){
	return myRobot->getPose();
}
void GotoGoal::shutdown(){
	Aria::shutdown();
	Aria::exit(0);
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
void GotoGoal::cancelGoal(){
	gotoGoalAction.cancelGoal();
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
bool detect(Mat frame, CascadeClassifier cascade) {
	std::vector<cv::Rect> ball;
	cascade.detectMultiScale(frame, ball, 1.1 , 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
	int sizeball = ball.size();
	if (sizeball != 1)
		 return false;
	 vector<Rect>::const_iterator r = ball.begin();
	 if (abs((float)r->width - (float)r->height) > delta || r->width < 30 || r->height < 30)
		 return false;
	 selection.x = r->x;
	 selection.y = r->y;
	 selection.width = r->x + (int)r->width;
	 selection.height = r->y + (int)r->height;
	 selection &= Rect(0, 0, image.cols, image.rows);
	 Point center( r->x + r->width/2, r->y + r->height/2 );
	 ellipse( frame, center, Size( r->width/2, r->height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
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
	if (abs(width - height) >  delta || width < 30 || height < 30)
		return false;
	ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
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
	if (x <= 200)
		return (0 - 10);
//		return -10;
	else if (x <= 440)
		return 0;
//	else
//		return 10;
	return 10;
}
bool follow(Mat hsv, Mat mask) {
	float vel = 0;
	float angle = 0;
	gotoGoal.enableDirectionCommand();
	if(trackObject(hsv, mask)) {
		float d = distance();
		if (d <= 250) {
			gotoGoal.setVel(20);
			gotoGoal.move(d - 250);
		} else if (d <= 300){
			gotoGoal.disableRobot();
			findObject =  1;
			cout<<"save image";
			imwrite("./image/ball.jpg", image);
//			return false;
		} else {
			vel = d * 0.7;
			vel = (int) (vel/50) * 50;
			if (vel > 200)
				vel = 200;
			gotoGoal.setVel(vel);
			gotoGoal.move(d - 250);
//						gotoGoal.move(20);
		}
		angle =  determindRotate();

		cout<<"khoang cach: "<<d<<"\tGoc quay: "<<angle<<"\t van toc = "<<vel<<endl;
		if (angle != 0) {
			gotoGoal.rotate(angle);
		}
	} else {
		checkObject = false;
		cout<< "Bat sai"<<endl;
	}
	return true;
}
void showWindows(){
	namedWindow( "threshold", 0 );
	namedWindow( "trackbar", 1 );
//	namedWindow( "Histogram", 0 );
	namedWindow( "main", 0 );
	createTrackbar( "Vmin", "trackbar", &vmin, 256, 0 );
	createTrackbar( "Vmax", "trackbar", &vmax, 256, 0 );
	createTrackbar( "Smin", "trackbar", &smin, 256, 0 );
}
bool showFrames(){
	imshow("main", image);
	imshow( "threshold", mask );
//	imshow( "Histogram", histimg );
	char c = (char)waitKey(10);
	if( c == 27 )
		return false;
	return true;
}
bool readFrame(VideoCapture cap) {
	cap >> frame;
	if( frame.empty() ){
		cout<<"error camera"<<endl;
		return false;
	}
	frame.copyTo(image);
	cvtColor(image, hsv, COLOR_BGR2HSV);
	int _vmin = vmin, _vmax = vmax;
	inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
			Scalar(180, 256, MAX(_vmin, _vmax)), mask);
	return true;
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
//	ArServerInfoRobot serverInfo(&server, &robot);
	ArServerFileToClient serverFileToClient(&server, ".");
	gotoGoal = GotoGoal(&robot, &sonar, &server, &serverFileToClient);//, &serverInfo, &serverHanlerCommands, &serverSimpleComUC);
	gotoGoal.init(argc, argv);
	gotoGoal.disableDirectionCommand();
//	gotoGoal.enableDirectionCommand();
//	while(gotoGoal.isStop());
//	gotoGoal.disableDirectionCommand();
	float angle = 0;
	VideoCapture cap;
	cap.open(0);
	Rect trackWindow;

	int hsize = 16;
	showWindows();
	CascadeClassifier c;
	c.load("cascade.xml");
	Mat hue, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	float vel = 0;
	ArPose* poseList = readPostitions("positions.txt");
	ArTime timer; //timer

	int i = 0;

	while(i < 25 ) {//&& !findObject) {

		gotoGoal.gotoGoal(poseList[i]);
		timer.setToNow();
		bool checkAchievedGoal = false;
		while (!gotoGoal.haveAchievedGoal()) {
			check = false;
//			cout<<"x = "<<gotoGoal.getPose().getX()<<"\t y = "<<gotoGoal.getPose().getY()<<endl;
			if (timer.mSecSince() > timeOut) {
				gotoGoal.cancelGoal();
				break;
			}
			readFrame(cap);
			if (!checkObject)
				checkObject = detect(frame, c);

			showFrames();
			if (checkObject) {
				cout <<"Phat hien doi tuong!!!!!!!!!!!!!!!"<<endl;
				check = true;
			}
			gotoGoal.enableDirectionCommand();
			while(checkObject) {
				readFrame(cap);
				follow(hsv, mask);
				showFrames();
			}

			if (!checkObject && check){
				cout <<"Bat dau quay!!!!!!!!!!!!!!!"<<endl;
				int turn = 0;
				gotoGoal.enableDirectionCommand();
				bool checkRotate = false;
				while (turn < 17 && !checkRotate) {//&& !check) {
					turn ++;
					gotoGoal.rotate(myAngle);
					cout <<"Quay "<<turn * myAngle<<" do"<<endl;
					gotoGoal.setVel(0);

					while(!gotoGoal.haveRotated()) {
						if (!readFrame(cap))
							break;
						if (!checkObject)
							checkObject = detect(frame, c);
						if (checkObject){
							checkRotate = true;
							break;
						}
						showFrames();
					}
				}
			}

			gotoGoal.disableDirectionCommand();
		}
		cout <<"Da den muc tieu!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
		gotoGoal.enableDirectionCommand();

		int turn = 0;

		while (turn < 17) {
			turn ++;
			gotoGoal.setVel(0);
			gotoGoal.rotate(myAngle);
			cout <<"Quay "<<turn * myAngle<<" do"<<endl;
			while(!gotoGoal.haveRotated()) {
				if (!readFrame(cap))
					break;
				if (!checkObject)
					checkObject = detect(frame, c);
				if (checkObject){
					check = true;
					break;
				}
			}
		}
		gotoGoal.enableDirectionCommand();
		while(checkObject) {
			readFrame(cap);
			follow(hsv, mask);
			showFrames();
		}
		i++;
		gotoGoal.disableDirectionCommand();
	}
	robot.waitForRunExit(0);
	gotoGoal.shutdown();
}

