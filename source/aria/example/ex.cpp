#include "Aria.h"
#include "Arnl.h"
#include "ArSonarLocalizationTask.h"
#include "ArSystemStatus.h"
using namespace std;

void addGoalDone(ArPose pose) {
	ArLog::log(ArLog::Normal, "x = %f, y = %f", pose.getX(), pose.getY());
}
int main (int argc, char** argv) {
	
	Aria::init();
	Arnl::init();
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
	robot.addRangeDevice(&sonarDev);
	
	//robot.runAsync(true);
	char fileDir[1024];
	  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(),
				 "examples");
	ArMap map("office.map");
//	  ArMap map(fileDir);


	// set it up to ignore empty file names (otherwise if a configuration omits
	// the map file, the whole configuration change will fail)
	map.setIgnoreEmptyFileName(true);
	// ignore the case, so that if someone is using MobileEyes or
	// MobilePlanner from Windows and changes the case on a map name,
	// it will still work.
	map.setIgnoreCase(true);
	
	ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);
	ArPathPlanningTask pathPlan(&robot, &sonarDev, &map);
	//locTask.localizeRobotAtHomeBlocking();
	robot.setVel(200);
	ArPose pose;
	//locTask.forceUpdatePose(pose);
	pathPlan.pathPlanToPose(ArPose(1000, 1000, 10000), true, true);
	ArGlobalFunctor1<ArPose> add(&addGoalDone);// = new ArGlobalFunctor1(&addGoalDone);
	pathPlan.addGoalDoneCB(&add);

	robot.runAsync(true);
	robot.enableMotors();
	//robot.lock();
	
	//robot.unlock();
	
	
	/*
	while(true) {
	//while (robot.blockingConnect()){
		//robot.lock();
		//ArPose pose  = robot.getPose();
		//pose.setX(100);
		//robot.moveTo(pose);
		//t = robot.getLastOdometryTime();
		//int a = interp.getPose(t, &pose);
		ArLog::log(ArLog::Normal, "x = %f \t y = %f\n", pose.getX(), pose.getY());
		//robot.unlock();
	} 
	* */
}
