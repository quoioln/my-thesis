#include "Aria.h"
#include "Arnl.h"
#include "ArPathPlanningTask.h"
#include "ArSonarLocalizationTask.h"
int main (int argc, char** argv) {
	Arnl::init();
	Aria::init();

	ArRobot robot;
	ArArgumentParser parser(&argc, argv);

	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	ArAnalogGyro gyro(&robot);
	ArSimpleConnector simpleConnector(&parser);

	simpleConnector.parseArgs();

	// Check for --help
	if (!parser.checkHelpAndWarnUnparsed())
	{
	printf("\nUsage: simpleDemo [-map <map file>] [other options]\n\nIf -map not given, use map from ARNL configuration file (%s).\n\n", Arnl::getTypicalParamFileName());
	simpleConnector.logOptions();
	Aria::exit(4);
	}

	ArSonarDevice sonarDev;
	ArBumpers bumpers;
	robot.lock();
	robot.addRangeDevice(&sonarDev);
	robot.addRangeDevice(&bumpers);
	robot.unlock();
	parser.loadDefaultArguments();

	ArMap map;
	map.setIgnoreEmptyFileName(true);
	ArLog::log(ArLog::Normal, "map name = %s",map.getFileName() );

	// Read ARNL parameter file
	  Aria::getConfig()->useArgumentParser(&parser);
	  if (Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
	  {
	    printf("Loaded configuration file %s\n", Arnl::getTypicalParamFileName());
	  }
	  else
	  {
	    printf("Trouble loading configuration file, exiting\n");
	    Aria::exit(5);
	  }

	  // Connect to the robot
	  if (!simpleConnector.connectRobot(&robot))
	  {
	    printf("Could not connect to robot... exiting\n");
	    Aria::shutdown();
	    Aria::exit(1);
	  }
	ArPathPlanningTask pathTask(&robot, &sonarDev, &map);
	ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);

	ArActionPlanAndMoveToGoal action(500, 200, &pathTask, NULL, &sonarDev);

	// Action to slow down robot when localization score drops but not lost.
	ArActionSlowDownWhenNotCertain actionSlowDown(&locTask);
	pathTask.getPathPlanActionGroup()->addAction(&actionSlowDown, 140);

	// Action to stop the robot when localization is "lost" (score too low)
	ArActionLost actionLostPath(&locTask, &pathTask);
	pathTask.getPathPlanActionGroup()->addAction(&actionLostPath, 150);

	robot.lock();
	pathTask.addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);
	ArForbiddenRangeDevice forbidden(&map);
//	ArLine
//	robot.addRangeDevice(&forbidden);
	pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);
	ArGlobalReplanningRangeDevice replanDev(&pathTask);
	robot.unlock();

	robot.runAsync(true);
	robot.lock();
	locTask.localizeRobotAtHomeBlocking();
//	pathTask.
	ArPose pose;
	pose.setX(4000);
	pose.setY(-1000);
	pose.setTh(9);
	robot.unlock();
	pathTask.pathPlanToPose(pose, true, true);
	robot.enableMotors();
	robot.waitForRunExit();
	Aria::shutdown();
}
