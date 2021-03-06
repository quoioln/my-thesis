
/*

MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Copyright (C) 2004, 2005, ActivMedia Robotics LLC.
Copyright (C) 2006, 2007, MobileRobots Inc.
All Rights Reserved

MobileRobots Inc does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
MobileRobots
19 Columbia Drive
Amherst, NH 03031
800-639-9481

*/

#include "Aria.h"
#include "ArNetworking.h"
#include "ArLocalizationTask.h"
#include "Arnl.h"

class SimpleTask
{
public:
  SimpleTask(ArRobot *robot, ArPathPlanningTask *pathTask);
  virtual ~SimpleTask();
  void task(void);
  void goalDone(ArPose pose);
protected:
  ArRobot *myRobot;
  ArPathPlanningTask *myPathTask;

  int myState;
  bool myGoalDone;
  ArPose myHome;
  ArFunctorC<SimpleTask> myTaskCB;
  ArFunctor1C<SimpleTask, ArPose> myGoalDoneCB;
};

SimpleTask::SimpleTask(ArRobot *robot, ArPathPlanningTask *pathTask) :
  myTaskCB(this, &SimpleTask::task),
  myGoalDoneCB(this, &SimpleTask::goalDone)
{
  myRobot = robot;
  myPathTask = pathTask;

  // trigger the first start off
  myGoalDone = true;
  myState = 0;
  myHome = robot->getPose();

  myRobot->addUserTask("simpleTask", 50, &myTaskCB);
  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalDoneCB);
}

SimpleTask::~SimpleTask()
{

}

void SimpleTask::goalDone(ArPose pose)
{
  myGoalDone = true;
}

void SimpleTask::task(void)
{
  if (!myGoalDone)
    return;

  myGoalDone = false;
  myState++;
  if (myState == 1)
    myPathTask->pathPlanToPose(ArPose(1000, 0, 0), true);
  //else if (myState == 1)
  //myPathTask->pathPlanToGoal("Goal1");
  else
  {
    myState = 0;
    myPathTask->pathPlanToPose(myHome, true);
  }
}

int
main(int argc, char *argv[])
{
  // Initialize location of Aria, Arnl and their args.
  Aria::init();
  Arnl::init();

  // The robot object
  ArRobot robot;
#ifndef SONARNL
  // The laser object
  ArSick sick(181, 361);
#endif

  // Parse them command line arguments.
  ArArgumentParser parser(&argc, argv);

  // Set up our simpleConnector
  ArSimpleConnector simpleConnector(&parser);

  // Load default arguments for this computer
  parser.loadDefaultArguments();

  // Parse its arguments for the simple connector.
  simpleConnector.parseArgs();

  // sonar, must be added to the robot, for teleop and wander
  ArSonarDevice sonarDev;
  // add the sonar to the robot
  robot.addRangeDevice(&sonarDev);

  ArMap arMap;
#ifndef SONARNL
  // Initialize the localization
  ArLocalizationTask locTask(&robot, &sick, &arMap);
  // Make the path task
  ArPathPlanningTask pathTask(&robot, &sick, &sonarDev, &arMap);
#else
  // Initialize the localization
  ArSonarLocalizationTask locTask(&robot, &sonarDev, &arMap);
  // Make the path task
  ArPathPlanningTask pathTask(&robot, NULL, &sonarDev, &arMap);
#endif


  // Stop the robot as soon as localization fails.
  ArFunctor1C<ArPathPlanningTask, int>
  locaFailed(&pathTask, &ArPathPlanningTask::trackingFailed);
  locTask.addFailedLocalizationCB(&locaFailed);

  // Read in param files.
  Aria::getConfig()->useArgumentParser(&parser);
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    printf("Trouble loading configuration file, exiting\n");
    exit(1);
  }
  // Warn about unknown params.
  if (!parser.checkHelpAndWarnUnparsed())
  {
    printf("\nUsage: %s -map mapfilename\n\n", argv[0]);
    simpleConnector.logOptions();
    exit(2);
  }

  // Our server
  ArServerBase server;

  // First open the server up
  if (!server.open(7272))
  {
    printf("Could not open server port\n");
    exit(1);
  }

  // Connect the robot
  if (!simpleConnector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }

  robot.com2Bytes(31, 14, 0);
  robot.com2Bytes(31, 15, 0);
  ArUtil::sleep(100);
  robot.com2Bytes(31, 14, 1);
  robot.com2Bytes(31, 15, 1);
  robot.enableMotors();
  robot.clearDirectMotion();

#ifndef SONARNL
  // Set up the laser before handing it to the laser mode
  simpleConnector.setupLaser(&sick);

  // Add the laser to the robot
  robot.addRangeDevice(&sick);
#endif

  // Start the robot thread.
  robot.runAsync(true);

#ifndef SONARNL
  // Start the laser thread.
  sick.runAsync();

  // Connect the laser
  if (!sick.blockingConnect())
  {
    printf("Couldn't connect to sick, exiting\n");
    Aria::shutdown();
    return 1;
  }
#endif

  ArUtil::sleep(300);

  // If you want to set the number of samples change the line below
  locTask.setNumSamples(2000);
  // Localize the robot to home
  if(locTask.localizeRobotAtHomeBlocking())
  {
    printf("Successfully localized at home.\n");
  }
  else
  {
    printf("WARNING: Unable to localize at home position!\n");
  }


  robot.lock();
  // attach stuff to the server
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoPath serverInfoPath(&server, &robot, &pathTask);
  ArServerInfoLocalization serverInfoLocalization(&server, &robot, &locTask);

#ifndef SONARNL
  // Set it up to handle maps.
  ArServerHandlerMap serverMap(&server, &arMap, ArServerHandlerMap::POINTS);
#else
  ArServerHandlerMap serverMap(&server, &arMap, ArServerHandlerMap::LINES);
#endif

  // Set up a service that allows the client to monitor the communication
  // between the robot and the client.
  //
  ArServerHandlerCommMonitor serverCommMonitor(&server);


  //ArServerModeGoto modeGoto(&server, &robot, &pathTask);
  //ArServerModeStop modeStop(&server, &robot);
  //ArServerModeDrive modeDrive(&server, &robot);

  SimpleTask simpleTask(&robot, &pathTask);

  robot.unlock();
  // Read in param files.
  Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName());

  // Now let it spin off in its own thread
  server.run();
  exit(0);

}
