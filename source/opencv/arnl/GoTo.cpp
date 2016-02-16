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
//	robot.lock();
	robot.comInt(ArCommands::ENABLE, 1);
	robot.addRangeDevice(&sonarDev);
//	ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);
	ArActionGoto gotoPoseAction("goto", ArPose(0.0, 0.0, 0.0), 200);
	ArActionAvoidFront avoidFront("avoid front");
	robot.addAction(&gotoPoseAction, 50);
	robot.addAction(&avoidFront, 60);

//	ArPathPlanningTask pathPlan(&robot, &sonarDev, &map);
//	locTask.localizeRobotAtHomeBlocking();
//	ArGlobalFunctor1<ArPose> add(&addGoalDone);// = new ArGlobalFunctor1(&addGoalDone);
//	pathPlan.addGoalDoneCB(&add);
//	ArActionPlanAndMoveToGoal gotoGoal (200, 10, pathPlan, NULL, &sonarDev);

//	pathPlan.runAsync();
//	pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true);

//	robot.unlock();
	robot.moveTo(ArPose(0,0,0));
	gotoPoseAction.setGoal(ArPose(3000, 0, 0));
	ArTime start; //timer
	start.setToNow();//start timer
//	start.
	while (!gotoPoseAction.haveAchievedGoal()) {

		//if (start.mSecSince() == 4000){
			//ArLog::log(ArLog::Normal ,"time = %e", start.mSecSince());
//			gotoPoseAction.cancelGoal();
//			robot.lock();

//			robot.setDeltaHeading(-90);
//			robot.unlock();
//			;
//			gotoPoseAction.
//			robot.stop();
			//robot.disableMotors();
//			robot.lock();
//			robot.clearDirectMotion();
//			robot.unlock();
//			robot.unlock();
			ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
			ArUtil::sleep(2000);
			//robot.enableMotors();
//			robot.lock();
//			robot.addAction(&gotoPoseAction, 50);
//			gotoPoseAction.setGoal(ArPose(1000, 1000, 0));
//			robot.unlock();
//			gotoPoseAction.activate();
		//}
	}
	gotoPoseAction.setGoal(ArPose(3000, -1000, 0));
	while(!gotoPoseAction.haveAchievedGoal()) {
		
		ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
		ArUtil::sleep(2000);
	}
//		ArUtil::sleep(4000);
	gotoPoseAction.setGoal(ArPose(0, -1000, 0));
	while(!gotoPoseAction.haveAchievedGoal()) {
		
		ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
		ArUtil::sleep(2000);
	}
	gotoPoseAction.setGoal(ArPose(0, 0, 0));
	while(!gotoPoseAction.haveAchievedGoal()) {
		ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
		ArUtil::sleep(2000);
	}
	printf ("OK");

//		robot.
//		robot.lock();
//		printf ("x = %.2f, y = %.2f", robot.getX(), robot.getY());
//		robot.unlock();
//	}
//	robot.lock();
//	robot.setVel(200);
//	robot.unlock();
	//while (pathPlan.getRunningWithLock())
		//ArUtil::sleep(1000);

//	ArPose pose;
//	locTask.forceUpdatePose(pose);
	
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
