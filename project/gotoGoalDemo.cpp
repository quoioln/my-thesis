#include<iostream>
#include<fstream>
#include "Aria.h"
#include "Arnl.h"
#include "ArSonarLocalizationTask.h"
#include "ArSystemStatus.h"
#define ARRAY_SIZE(array)(sizeof(array[0])/sizeof(array))
using namespace std;

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
	/*
	for (int i = 0 ; i < 4; i++) {
			cout <<"("<<postitionList[i].getX()<<", "<<postitionList[i].getY()<<")"<<endl;
	}
	*/
	return postitionList;
}
int main (int argc, char** argv) {

	Aria::init();
	ArRobot robot;
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	//ArAnalogGyro gyro = new
//	ArAnalogGyro gyro(&robot);
	if (!robotConnector.connectRobot()) {
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if(parser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}

	ArSonarDevice sonarDev;

//	ifstream is("positions.txt");

//	int n;
//	is >> n;

//	ArPose* poseList[] = new ArPose[n];
//	robot.runAsync(true);

//	ArMap map("office.map");
	// set it up to ignore empty file names (otherwise if a configuration omits
	// the map file, the whole configuration change will fail)
//	map.setIgnoreEmptyFileName(true);
	// ignore the case, so that if someone is using MobileEyes or
	// MobilePlanner from Windows and changes the case on a map name,
	// it will still work.
//	map.setIgnoreCase(true);

	ArPose* poseList = readPostitions("positions.txt");

	robot.runAsync(true);
	robot.enableMotors();
	robot.moveTo(ArPose(0,0,0));
	//robot.lock();
	robot.comInt(ArCommands::ENABLE, 1);
	robot.addRangeDevice(&sonarDev);
	//robot.unlock();
//	ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);
	ArActionGoto gotoPoseAction("goto", ArPose(0, 0, 0), 200);
	ArActionAvoidFront avoidFront("avoid front");
	ArActionStallRecover stallRecover("stall recover");
	robot.addAction(&gotoPoseAction, 50);
	robot.addAction(&avoidFront, 60);
	robot.addAction(&stallRecover, 70);
	robot.moveTo(ArPose(0,0,0));
//	gotoPoseAction.setGoal(ArPose(3000, 0, 0));
//	ArTime start; //timer
//	start.setToNow();//start timer
	int length = ARRAY_SIZE(poseList);
	for (int i = 0; i < length; i++) {
		gotoPoseAction.setGoal(poseList[i]);
		while (!gotoPoseAction.haveAchievedGoal()) {
			ArLog::log(ArLog::Normal, "goal(%.2f, %0.2f) x = %.2f, y = %.2f", poseList[i].getX(), poseList[i].getY(), robot.getX(), robot.getY());
		}

//		avoidFront.deactivate();
//		gotoPoseAction.deactivate();
//		robot.comInt(ArCommands::ENABLE, 1);
//		robot.disableMotors();
//		robot.lock();
//		robot.stop();
//		robot.stop();
		/*
		robot.lock();
		robot.setDeltaHeading(90);
		robot.unlock();
		while(!robot.isHeadingDone());
		robot.clearDirectMotion();

		robot.lock();
		robot.setDeltaHeading(90);
		robot.unlock();
		while(!robot.isHeadingDone());
		robot.clearDirectMotion();
		robot.lock();
		robot.setDeltaHeading(90);
		robot.unlock();
		while(!robot.isHeadingDone());
		robot.clearDirectMotion();

		*/
		/*
		int t = 1;
		while(t * 30 <= 360) {
			robot.lock();
			robot.setDeltaHeading(45);
			robot.unlock();
			t++;
			
		}
	*/
		//ArUtil::sleep(200);
//		robot.lock();
//		robot.setDeltaHeading(90);
//		robot.unlock();
		ArUtil::sleep(200);
//				robot.enableMotors();
//		robot.comInt()
//		robot.clearDirectMotion();
//		int t = 1;
//		robot.lock();
//		robot.stop();
//		robot.setVel(0);
//		robot.unlock();
		/*
		while(t * 30 <= 360) {
			robot.lock();

//			cout<<t<<endl;
			robot.unlock();
			t ++;
		}
		*/
		avoidFront.activate();
		gotoPoseAction.activate();

	}

	ArUtil::sleep(2000);
	Aria::shutdown();
}
