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
	ArLog::log(ArLog::Normal, "OK");
	char fileDir[1024];
	ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), "examples");
	ArLog::log(ArLog::Normal, "Installation directory is: %s\nMaps directory is: %s\n", Aria::getDirectory(), fileDir);
	//		strcpy(fileDir, "columbia.map");
	ArLog::log(ArLog::Normal, "file Maps directory is: %s\n",fileDir);
	ArMap map(fileDir);
	map.readFile("columbia.map");
	// set it up to ignore empty file names (otherwise the parseFile
	// on the config will fail)
	map.setIgnoreEmptyFileName(true);



	robot.addRangeDevice(&sonarDev);
	// set home pose
	robot.moveTo(ArPose(0,0,0));

	ArPathPlanningTask pathPlan(&robot, &sonarDev, &map);
	ArActionGoto gotoPoseAction("goto");
//	gotoPoseAction.activate();
	pathPlan.getPathPlanActionGroup()->addAction(&gotoPoseAction, 50);
//	pathPlan.getPathPlanAction()->activate();
	ArForbiddenRangeDevice forbidden(&map);
	robot.addRangeDevice(&forbidden);
	pathPlan.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);
//	pathPlan.planAndSetupAction(ArPose(0, 0, 0));
//	pathPlan.
/*
	if (pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true))
		ArLog::log(ArLog::Normal, "OK");
	else
		ArLog::log(ArLog::Normal, "FAILED");
*/
	// create functor
	ArGlobalFunctor1<ArPose> goalDone(&addGoalDone);// = new ArGlobalFunctor1(&addGoalDone);
	ArGlobalFunctor1<ArPose> goalFail(&addGoalFailed);
	ArGlobalFunctor1<ArPose> goalFinished(&addGoalFinished);
	ArGlobalFunctor1<ArPose> goalInterrupted(&addGoalInterrupted);

	// add functor
	pathPlan.addGoalDoneCB(&goalDone);
	pathPlan.addGoalFailedCB(&goalFail);
	pathPlan.addGoalFinishedCB(&goalFinished);
	pathPlan.addGoalInterruptedCB(&goalInterrupted);

	robot.runAsync(true);
	robot.enableMotors();
	pathPlan.runAsync();

	ArPathPlanningTask::PathPlanningState state = pathPlan.getState();
//	while(!pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true));
	pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true);
//	char* text = new char[512];
//	pathPlan.getStatusString(text, sizeof(text));
//	 printf("Planning status: %s.\n", text);
//	while(pathPlan.pathPlanToPose(ArPose(1000, 1000, 0), true, true));
	Aria::shutdown();
//	Arnl::s;
	Aria::exit(0);
}
