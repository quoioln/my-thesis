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

	for (int i = 0 ; i < 4; i++) {
			cout <<"("<<postitionList[i].getX()<<", "<<postitionList[i].getY()<<")"<<endl;
	}

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
	ArActionTurn actionTurn;
	robot.addAction(&gotoPoseAction, 50);
	robot.addAction(&avoidFront, 60);

	ArActionAvoidSide avoidSide;
	ArActionLimiterForwards	limitForwards;
	 ArActionConstantVelocity constrant("Constant Velocity", 400);
	robot.addAction(&stallRecover, 70);
//	robot.addAction(&constrant, 20);
//	robot.addAction(&actionTurn, 40);
//	robot.addAction(&avoidSide, 60);
//	robot.addAction(&limitForwards, 80);
//	robot.moveTo(ArPose(0,0,0));
//	gotoPoseAction.setGoal(ArPose(3000, 0, 0));
//	ArTime start; //timer
//	start.setToNow();//start timer
	ArPose* poseList = readPostitions("positions.txt");
	int length = ARRAY_SIZE(poseList);
	cout <<"size of = "<<sizeof(poseList)<<endl;
	for (int i = 0; i < 28; i++) {

		gotoPoseAction.setGoal(poseList[i]);
		while (!gotoPoseAction.haveAchievedGoal()) {
			ArLog::log(ArLog::Normal, "goal(%.2f, %0.2f) x = %.2f, y = %.2f", poseList[i].getX(), poseList[i].getY(), robot.getX(), robot.getY());
		}
//		cout <<"("<<poseList[i].getX()<<", "<<poseList[i].getY()<<")";
	}
	robot.waitForRunExit();
	ArUtil::sleep(2000);
	Aria::shutdown();
}
