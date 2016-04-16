#include<iostream>
#include<fstream>
#include "Aria.h"
#include "Arnl.h"
//#include "ArSonarLocalizationTask.h"
#include "ArPathPlanningTask.h"
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
	Arnl::init();
	Aria::init();
	ArRobot robot;
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	//ArAnalogGyro gyro = new
	ArAnalogGyro gyro(&robot);
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
//	ArMap map;
	// set it up to ignore empty file names (otherwise if a configuration omits
	// the map file, the whole configuration change will fail)
//	map.setIgnoreEmptyFileName(true);
	// ignore the case, so that if someone is using MobileEyes or
	// MobilePlanner from Windows and changes the case on a map name,
	// it will still work.
//	map.setIgnoreCase(true);
//
//	if(!map.readFile("boMon2.map"))
//		ArLog::log(ArLog::Normal, "Can not open map");
	// Set up where we'll look for files
	  char fileDir[1024];
	  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(),
	             "examples");
	  ArLog::log(ArLog::Normal, "Installation directory is: %s\nMaps directory is: %s\n", Aria::getDirectory(), fileDir);

	  // Set up the map, this will look for files in the examples
	  // directory (unless the file name starts with a /, \, or .
	  // You can take out the 'fileDir' argument to look in the current directory
	  // instead
	  ArMap map(fileDir);
	  // set it up to ignore empty file names (otherwise the parseFile
	  // on the config will fail)
	  map.setIgnoreEmptyFileName(true);

	ArPathPlanningTask pathTask(&robot, &sonarDev, &map);

	ArBumpers bumpers;
	robot.lock();
	robot.addRangeDevice(&bumpers);
	pathTask.addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);
	ArForbiddenRangeDevice forbidden(&map);
	robot.addRangeDevice(&forbidden);
	pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);
	ArGlobalReplanningRangeDevice replanDev(&pathTask);

	ArActionPlanAndMoveToGoal planAndMove(200, 20, &pathTask,NULL, &sonarDev);
//	planAndMove.
	robot.unlock();

	ArPose* poseList = readPostitions("positions.txt");

	robot.runAsync(true);
	robot.enableMotors();
	robot.moveTo(ArPose(0,0,0));
	//robot.lock();
	robot.comInt(ArCommands::ENABLE, 1);
	robot.addRangeDevice(&sonarDev);
	//robot.unlock();
//	ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);
	robot.moveTo(ArPose(0,0,0));
	int length = ARRAY_SIZE(poseList);
	for (int i = 0; i < length; i++) {
		pathTask.pathPlanToPose(poseList[i], true, true);

	}

	ArUtil::sleep(2000);
	Aria::shutdown();
}
