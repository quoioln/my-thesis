#include "Aria.h"
#include "Arnl.h"
#include "ArSonarLocalizationTask.h"
#include "ArSystemStatus.h"
using namespace std;

void addGoalDone(ArPose pose) {
	ArLog::log(ArLog::Normal, "x = %f, y = %f", pose.getX(), pose.getY());
}
void addGoalFailed(ArPose pose){
	ArLog::log(ArLog::Normal, "Fail at x = %f, y = %f", pose.getX(), pose.getY());
}

void addGoalFinished(ArPose pose) {
	ArLog::log(ArLog::Normal, "Finished at x = %f, y = %f", pose.getX(), pose.getY());
}

void addGoalInterrupted(ArPose pose) {
	ArLog::log(ArLog::Normal, "Interrupted at x = %f, y = %f", pose.getX(), pose.getY());
}
int main (int argc, char** argv) {
	
	Aria::init();
	Arnl::init();
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
	

//	robot.runAsync(true);
	ArLog::log(ArLog::Normal, "OK");
	ArMap map("office.map");
	// set it up to ignore empty file names (otherwise if a configuration omits
	// the map file, the whole configuration change will fail)
//	map.setIgnoreEmptyFileName(true);
	// ignore the case, so that if someone is using MobileEyes or
	// MobilePlanner from Windows and changes the case on a map name,
	// it will still work.
//	map.setIgnoreCase(true);


	robot.runAsync(true);

//	robot.lock();
//	robot.comInt(ArCommands::ENABLE, 1);
	robot.addRangeDevice(&sonarDev);
//	ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);
//	ArActionAvoidFront avoidFront("avoid front obstacles");
//	ArActionGoto gotoPoseAction("goto");
//	robot.addAction(&gotoPoseAction, 50);
//	robot.addAction(&avoidFront, 60);
//	robot.moveTo(ArPose(0,0,0));
	ArPathPlanningTask pathPlan(&robot, &sonarDev, &map);
//	pathPlan.getRun
//	locTask.localizeRobotAtHomeBlocking();
	ArGlobalFunctor1<ArPose> goalDone(&addGoalDone);// = new ArGlobalFunctor1(&addGoalDone);
	ArGlobalFunctor1<ArPose> goalFail(&addGoalFailed);
	ArGlobalFunctor1<ArPose> goalFinished(&addGoalFinished);
	ArGlobalFunctor1<ArPose> goalInterrupted(&addGoalInterrupted);
	pathPlan.addGoalDoneCB(&goalDone);
	pathPlan.addGoalFailedCB(&goalFail);
	pathPlan.addGoalFinishedCB(&goalFinished);
	pathPlan.addGoalInterruptedCB(&goalInterrupted);

//	ro
//	ArActionPlanAndMoveToGoal gotoGoal (200, 10, pathPlan, NULL, &sonarDev);

//	pathPlan.runAsync();
	robot.enableMotors();
//	while(!pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true));
	pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true);
//	pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true)
	pathPlan.startPathPlanToLocalPose(true);
//	pathPlan.getPathPlanActionGroup()->activate();
//	pathPlan.getCurrentGoal()
//	pathPlan.is
//	robot.unlock();
	/*
	gotoPoseAction.setGoal(ArPose(10000, 15000, 0));
	while (!gotoPoseAction.haveAchievedGoal()) {
		robot.lock();
		printf ("x = %.2f, y = %.2f", robot.getX(), robot.getY());
		robot.unlock();
	}
	*/
//	robot.lock();
//	robot.setVel(200);
//	robot.unlock();
	while (pathPlan.endPathPlanToLocalPose(true));
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
