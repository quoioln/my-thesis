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

/*

ActivMedia Robotics Navigation and Localization (ARNL)
Copyright (C) 2004, ActivMedia Robotics, LLC
Copyright (C) 2005, 2006 MobileRobots, Inc.

MobileRobots, Inc. does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.


*/
/*!****************************************************************************
 *
 * File: advanced.cpp
 *
 * Function: This is an advanced example of how to use ARNL, if you are just
 *           trying to get things to work or not do complicated things do
 *           not use this example but instead use the other examples.
 *
 *           The program will take as input a .map file and use it to
 *           start a localization task which will continously keep the
 *           robot localized in the map if correctly initialized..  It
 *           also starts a pathplanning task which can plan safe paths
 *           to goals after it is initialized.  The main class
 *           Advanced holds all the related classes in a single place.
 *
 *           This program does not contain any server classes, interact
 *           with this program through the terminal via command keys,
 *           not using MobileEyes.
 *
 * User Hints: To customize this localization and path planning module for
 *             user applications, Use all the functions called in the
 *             main() to initialize the two tasks. Then, localize and path plan
 *             using the functions shown in interact()
 *
 ****************************************************************************/
#include "Arnl.h"
#include "Aria.h"
#include "ArNetworking.h"

/*!
  @class Advanced.
  @brief Class holds details of the localization and path planning tasks, with callbacks, and provides an interface around them
*/
class Advanced
{
public:

  /// Base Constructor.
  Advanced(ArRobot *robot, ArSick *sick, ArSonarDevice *sonar);
  /// Base Destructor.
 ~Advanced(void);

  /// Start the localization thread.
  bool   initializeLocalizationTask(char* mn);
  /// Set robots current pose in the task data structure.
  bool   setTaskCurrentPose(double rx, double ry, double rt);
  /// Initialize the samples and run localize at given pose.
  bool   localizeAtSetPose(ArPose pose);
  /// Get the computed best goodness score.
  double getLocalizationScore(void);
  /// Failed tracking callback.
  void   trackingFailed(int tf);

  /// Set up the path planning.
  bool   initializePathPlanningTask(void);
  /// Plan a A* path from from to to.
  bool   pathPlanFromCurrentToDest(ArPose to, bool headflag);
  /// Path to goal success callback.
  void   goalDone(ArPose goal);
  /// Path to goal failure callback.
  void   goalFailed(ArPose goal);

  /// Shutdown the system.
  void   shutDown(void);
  /// Helper for goals.
  void   getAllGoals(ArMap* ariamap);
  /// Helper for homes
  void   getAllRobotHomes(ArMap* ariamap);

public:

  ArRobot*                  myRobot;
  ArSick*                   mySick;
  ArSonarDevice*            mySonar;
  ArMap*                    myMap;

#ifndef SONARNL
  ArLocalizationTask*       myLocaTask;
#else
  ArSonarLocalizationTask*       myLocaTask;
#endif

  ArPathPlanningTask*       myPathPlanningTask;

  std::list<ArMapObject*>   myGoalList;
  std::list<ArMapObject*>   myHomeList;
};


#define DEFAULT_NO_SAMPLES 2000
#define LOCA_PASS_THRESHOLD 0.20

#define USAGE "Usage: advanced -map mapfilename\n"

int noOfSamples = DEFAULT_NO_SAMPLES;

// Global container.
Advanced* advancedptr;
// Var for interaction.
ArKeyHandler keyHandler;
static bool roundRobinFlag = false;
/*!
 * Basic Constructor.
 *
 */
Advanced::Advanced(ArRobot *robot, ArSick *sick, ArSonarDevice *sonar)
{
  myRobot            = robot;
  mySick             = sick;
  mySonar            = sonar;

  myLocaTask         = NULL;
  myPathPlanningTask = NULL;
}
/*!
 * Basic Destructor.
 *
 */
Advanced::~Advanced(void)
{
  if(myLocaTask!=NULL)
    delete myLocaTask;
  if(myPathPlanningTask!=NULL)
    delete myPathPlanningTask;
}
/*!
 * Starts the localization thread going.
 *
 * @return true if successful.
 */
bool
Advanced::initializeLocalizationTask(char* mapname)
{
  // Start the localization task thread. This is heart of the system.
#ifndef SONARNL
  if(myRobot && mySick)
    myLocaTask = new ArLocalizationTask(myRobot, mySick, mapname);
#else
  if(myRobot && mySonar)
    myLocaTask = new ArSonarLocalizationTask(myRobot, mySonar, mapname);
#endif

  if(!myLocaTask){
    printf("Cannot allocate memory for Localization Task\n");
    return false;
  } else {
    // No of samples.
    myLocaTask->setNumSamples(noOfSamples);
    return true;
  }
}
/*!
 * Calls the localization task function around a given set pose of the robot.
 *
 * @return true if successful else false.
 */
bool
Advanced::localizeAtSetPose(ArPose pose)
{
  if(!myLocaTask){
    return false;
  }else{
    //
    // Jump in the start of the localization thread.
    //
    int ns      = myLocaTask->getNumSamples();
    double xstd = 200.0;
    double ystd = 200.0;
    double tstd = 30.0;
    return myLocaTask->localizeRobotInMapInit(pose, ns,
                          xstd, ystd, tstd,
                          LOCA_PASS_THRESHOLD);
  }
}
/*!
 * Returns the goodness of the last localization result. Fraction of the
 * matched range points to the total no of range points.
 *
 * @return A measure of the goodness of the map seen by the sensor and the
 *         robots pose.
 *
 */
double
Advanced::getLocalizationScore(void)
{
  return myLocaTask->getLocalizationScore();
}
/*!
 * This is the called when tracking fails.
 *
 * @param timesfailed: No of times the localization failed.
 */
void
Advanced::trackingFailed(int timesfailed)
{
  //
  // Probably should be doing something more than this in real use.
  //
  printf("\07Tracking Failed:Main\n");
  myPathPlanningTask->trackingFailed(timesfailed);
}

/*!
 * Sets up for path planning.
 *
 * @return true if successful.
 */
bool
Advanced::initializePathPlanningTask(void)
{
#ifdef SONARNL
  if(myRobot && mySonar && myMap)
    myPathPlanningTask = new ArPathPlanningTask(myRobot, mySonar,
                        myMap);
#else
  if(myRobot && mySick && mySonar && myMap)
    myPathPlanningTask = new ArPathPlanningTask(myRobot, mySick, mySonar,
                        myMap);
#endif

  if(!myPathPlanningTask){
    return false;
  }else{
    myRobot->lock();
    myRobot->enableMotors();
    myRobot->unlock();
    return true;
  }
}
/*!
 * Plans a collision free path  from current pose to the destination.
 *
 * @param to: The destination pose.
 * @param headflag: Flag to indicate if robot needs to orient after
 *                  reaching goal.
 *
 * @return true if successful.
 */
bool
Advanced::pathPlanFromCurrentToDest(ArPose to, bool headflag)
{
  //
  // Set the Goal and the flags to set off the path planning action.
  //
  if(myPathPlanningTask){
    return myPathPlanningTask->pathPlanToPose(to, headflag);
  }else{
    return false;
  }
}
/*!
 * This is the called when path to goal reached.
 *
 * @param goal: Goal it was assigned.
 */
void
Advanced::goalDone(ArPose goal)
{
  static int goalCount = 0;

  printf("goalDone:Main: %5.2f %5.2f %5.2f\07\n",
     goal.getX(), goal.getY(), goal.getTh());

  if(roundRobinFlag){
    if(myLocaTask->getInitializedFlag()){
      if(myGoalList.size()>0){
    ArMapObject* front = myGoalList.front();
    ArPose top = front->getPose();
    bool headingFlag = false;
    if(strcasecmp(front->getType(), "GoalWithHeading") == 0)
      headingFlag = true;
    else
      headingFlag = false;
    printf("Moving to next goal:%s %5.2f %5.2f %5.2f: %d poses %d Done\n",
           front->getName(),
           top.getX(), top.getY(), top.getTh(),
           myGoalList.size(), goalCount++);
    myGoalList.pop_front();
    myGoalList.push_back(front);

    myPathPlanningTask->pathPlanToPose(top, headingFlag);
      }
    }
  }else{
    printf("Localize the robot first\n");
  }
}
/*!
 * This is the called when path to goal fails
 *
 * @param goal: Goal it was assigned.
 */
void
Advanced::goalFailed(ArPose goal)
{
  printf("goalFailed:Main: %5.2f %5.2f %5.2f\07\n",
     goal.getX(), goal.getY(), goal.getTh());
}
/*!
 * Shuts down the system.
 *
 */
void
Advanced::shutDown(void)
{
  Aria::exit(0);
  //
  // Stop the path planning thread.
  //
  if(myPathPlanningTask){
    myPathPlanningTask->stopRunning();
//    delete myPathPlanningTask;
    printf("Stopped Path Planning Thread\n");
  }
  //
  // Stop the localization thread.
  //
  if(myLocaTask){
    myLocaTask->stopRunning();
    delete myLocaTask;
    printf("Stopped Localization Thread\n");
  }
  //
  // Stop the laser thread.
  //
  if(mySick)
  {
    mySick->lockDevice();
    mySick->disconnect();
    mySick->unlockDevice();
    printf("Stopped Laser Thread\n");
  }
  //
  // Stop the robot thread.
  //
  myRobot->lock();
  myRobot->stopRunning();
  myRobot->unlock();
  printf("Stopped Robot Thread\n");
  //
  // Exit Aria
  //
  Aria::shutdown();
  printf("Aria Shutdown\n");

}
/*!
 * Gets the list of goals.
 *
 * @param mapdata: Pointer to the map class.
 *
 * @return list of goal objs
 *
 */
void
Advanced::getAllGoals(ArMap* ariamap)
{
  myGoalList.clear();
  std::list<ArMapObject *>::iterator objIt;
  int i=0;
  ArMapObject* obj;
  for (objIt = ariamap->getMapObjects()->begin();
       objIt != ariamap->getMapObjects()->end();
       objIt++)
  {
    //
    // Get the forbidden lines and fill the occupancy grid there.
    //
    obj = (*objIt);
    if (strcasecmp(obj->getType(), "GoalWithHeading") == 0)
    {
      myGoalList.push_back(obj);
      ArPose pose = obj->getPose();
      printf("GoalWithHeading[%d] = %s : %5.2f %5.2f %5.2f\n",
         i++, obj->getName(),
         pose.getX(), pose.getY(), pose.getTh());
    }
    if (strcasecmp(obj->getType(), "Goal") == 0)
    {
      myGoalList.push_back(obj);
      ArPose pose = obj->getPose();
      printf("Goal[%d] = %s : %5.2f %5.2f %5.2f\n", i++, obj->getName(),
         pose.getX(), pose.getY(), pose.getTh());
    }
  }
}
/*!
 * Gets the list of home poses.
 *
 * @param mapdata: Pointer to the map class.
 *
 * @return list of home objects.
 *
 */
void
Advanced::getAllRobotHomes(ArMap* ariamap)
{
  myHomeList.clear();
  std::list<ArMapObject *>::iterator objIt;
  ArMapObject* obj;
  int i=0;
  for (objIt = ariamap->getMapObjects()->begin();
       objIt != ariamap->getMapObjects()->end();
       objIt++)
  {
    //
    // Get the forbidden lines and fill the occupancy grid there.
    //
    obj = (*objIt);
    if (strcasecmp(obj->getType(), "RobotHome") == 0)
    {
      myHomeList.push_back(obj);
      ArPose pose = obj->getPose();
      printf("RobotHome[%d] = %s : %5.2f %5.2f %5.2f\n", i++, obj->getName(),
         pose.getX(), pose.getY(), pose.getTh());
    }
    if (strcasecmp(obj->getType(), "Home") == 0)
    {
      myHomeList.push_back(obj);
      ArPose pose = obj->getPose();
      printf("Home[%d] = %s : %5.2f %5.2f %5.2f\n", i++, obj->getName(),
         pose.getX(), pose.getY(), pose.getTh());
    }
  }
}
/*!
 * Callback function for the l key. Localizes the robot at the home position.
 * Warning: Robot must be physically at the first home pose.
 */
void
lkeyCB(void)
{
  roundRobinFlag = false;

  static int once = 0;
  ArPose pose;
  if(!once && advancedptr->myHomeList.size()>0){
    once = 1;
    ArPose top = advancedptr->myHomeList.front()->getPose();
    printf("Localizing at %s %5.2f %5.2f %5.2f\n",
       advancedptr->myHomeList.front()->getName(),
       top.getX(), top.getY(), top.getTh());
    //
    // Set the pose to localize the robot about.
    //
    pose = top;
  } else {
    advancedptr->myRobot->lock();
    pose = advancedptr->myRobot->getPose();
    advancedptr->myRobot->unlock();
  }
  //
  // Do the localization at the current pose.
  //
  if(advancedptr->localizeAtSetPose(pose)){
    printf("Localized at current pose: Score: %5.2f out of 1.0\n",
       advancedptr->getLocalizationScore());
  }else {
    printf("Failed to localize\n");
  }
}
/*!
 * Callback function for the p key. Gets the robot to pick the next
 * goal point and plan to it.
 */
void
pkeyCB(void)
{
  roundRobinFlag = false;

  if(advancedptr->myLocaTask->getInitializedFlag()){
    if(advancedptr->myGoalList.size()>0){
      ArMapObject* front = advancedptr->myGoalList.front();
      ArPose top = front->getPose();
      bool headingFlag = false;
      if(strcasecmp(front->getType(), "GoalWithHeading") == 0)
    headingFlag = true;
      else
    headingFlag = false;
      printf("Path planing to goal %s %5.2f %5.2f %5.2f\n",
         front->getName(),
         top.getX(), top.getY(), top.getTh());
      advancedptr->myGoalList.pop_front();
      advancedptr->myGoalList.push_back(front);
      //
      // Setup the pathplanning task to this destination.
      //
      advancedptr->pathPlanFromCurrentToDest(top, headingFlag);
    }
  }else{
    printf("Localize the robot first\n");
  }
}
/*!
 * Callback function for the r key. Gets the robot to pick the next
 * goal point and plan to it. and keep going around the list.
 */
void
rkeyCB(void)
{
  if(roundRobinFlag == false){
    roundRobinFlag = true;
  }else{
    roundRobinFlag = false;
    return;
  }

  if(advancedptr->myLocaTask->getInitializedFlag()){
    if(advancedptr->myGoalList.size()>0){
      ArMapObject* front = advancedptr->myGoalList.front();
      ArPose top = front->getPose();
      bool headingFlag = false;
      if(strcasecmp(front->getType(), "GoalWithHeading") == 0)
    headingFlag = true;
      else
    headingFlag = false;
      printf("Path planing to goal %s %5.2f %5.2f %5.2f: %d poses\n",
         front->getName(),
         top.getX(), top.getY(), top.getTh(),
         advancedptr->myGoalList.size());
      advancedptr->myGoalList.pop_front();
      advancedptr->myGoalList.push_back(front);
      //
      // Setup the pathplanning task to this destination.
      //
      advancedptr->pathPlanFromCurrentToDest(top, headingFlag);
    }
  }else{
    printf("Localize the robot first\n");
  }
}
/*!
 * Callback function for the h key. Gets the robot to home pose.
 */
void
hkeyCB(void)
{
  roundRobinFlag = false;

  if(advancedptr->myLocaTask->getInitializedFlag()){
    if(advancedptr->myHomeList.size()>0){
      ArMapObject* front = advancedptr->myHomeList.front();
      ArPose top = front->getPose();
      printf("Homing to %s %5.2f %5.2f %5.2f\n",
         front->getName(),
         top.getX(), top.getY(), top.getTh());
      bool headingFlag = true;
      advancedptr->pathPlanFromCurrentToDest(top, headingFlag);
    }
  }else{
    printf("Localize the robot first\n");
  }
}
/*!
 * Callback function for the q key.
 */
void
quitCB(void)
{
  roundRobinFlag = false;
  keyHandler.restore();
  advancedptr->shutDown();
}
/*!
 * Interact with user on the terminal.
 */
void
interact()
{
  ArMap* ariamap = advancedptr->myMap;
  sleep(1);
  advancedptr->getAllGoals(ariamap);
  advancedptr->getAllRobotHomes(ariamap);

  /// MPL
//  lkeyCB();
  advancedptr->myLocaTask->localizeRobotAtHomeNonBlocking();
  //
  // Interact with user using keyboard.
  //
  ArGlobalFunctor lCB(&lkeyCB);
  ArGlobalFunctor pCB(&pkeyCB);
  ArGlobalFunctor hCB(&hkeyCB);
  ArGlobalFunctor rCB(&rkeyCB);
  ArGlobalFunctor qCB(&quitCB);
  ArGlobalFunctor escapeCB(&quitCB);

  keyHandler.addKeyHandler('l', &lCB);
  keyHandler.addKeyHandler('p', &pCB);
  keyHandler.addKeyHandler('h', &hCB);
  keyHandler.addKeyHandler('r', &rCB);
  keyHandler.addKeyHandler('q', &qCB);
  keyHandler.addKeyHandler(ArKeyHandler::ESCAPE, &escapeCB);

  printf("Put robot at RobotHome and press 'l' to localize first.\n\   
Press 'p' to move to the next goal\n\
Press 'h' to move to the first home\n\
Press 'r' to move to the goals in order\n\
Press 'q' to quit\n");   
  while (advancedptr->myLocaTask->getRunning() &&
     advancedptr->myPathPlanningTask->getRunning()){

    keyHandler.checkKeys();
    ArUtil::sleep(250);

    advancedptr->myRobot->lock();
    ArPose rpose = advancedptr->myRobot->getPose();
    double lvel = advancedptr->myRobot->getVel();
    double avel = advancedptr->myRobot->getRotVel();
    double volts = advancedptr->myRobot->getBatteryVoltage();
    advancedptr->myRobot->unlock();
    if(advancedptr->myLocaTask->getInitializedFlag()){
      printf("\r%5.2f %5.2f %5.2f: %5.2f %5.2f: %4.1f\r",
         rpose.getX(), rpose.getY(), rpose.getTh(), lvel, avel, volts);
      fflush(stdout);
    }
  }
}
/*!
 * Main function which initializes the localization tasks and pathplanning
 * tasks.
 *
 * @param argc No of command line args.
 * @param argv Array of command line args.
 *
 * @return 1 if successful.
 */
int
main(int argc, char *argv[])
{
  char* mapname;

  if(argc<2){
    printf("%s\n",USAGE);
    exit(1);
  }

  Aria::init();
  Arnl::init();

  // The robot object
  ArRobot robot;
#ifndef SONARNL
  // The laser object
  ArSick sick(181, 361);
#endif


  // Set up our simpleConnector
  ArSimpleConnector simpleConnector(&argc, argv);

  // Parse its arguments
  simpleConnector.parseArgs();

  // Parse them command line arguments.
  ArArgumentParser parser(&argc, argv);

  // Sonar, must be added to the robot, for teleop and wander
  ArSonarDevice sonarDev;
  // Add the sonar to the robot
  robot.addRangeDevice(&sonarDev);

  // Set up the robot for connecting
  if (!simpleConnector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }

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

  //
  // Set up the big holder.
  //
#ifndef SONARNL
  advancedptr = new Advanced(&robot, &sick, &sonarDev);
#else
  advancedptr = new Advanced(&robot, NULL, &sonarDev);
#endif
  //
  // Set up the callbacks for localization task.
  //
  ArFunctor1C<Advanced, int>
  failed(advancedptr, &Advanced::trackingFailed);
  //
  // Set up the callbacks for pathplanning task.
  //
  ArFunctor1C<Advanced, ArPose>
  goal_done(advancedptr, &Advanced::goalDone);
  ArFunctor1C<Advanced, ArPose>
  goal_failed(advancedptr, &Advanced::goalFailed);

  // Read in config file. Do this before creating the localization
  // and path planning tasks, so we can override certain config
  // parameters such as the map file name when we do so.
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    printf("Trouble loading configuration file, exiting\n");
    exit(1);
  }

  //
  // Get mapname and initialize localization task and pathplanning tasks.
  //
  if((mapname = parser.checkParameterArgument("-map"))){
    //
    // Start the localization thread. Should be done first.
    //
    if(!advancedptr->initializeLocalizationTask(mapname)){
      printf("Cannot start the localization task thread.\n");
      exit(1);
    }else{
      printf("Started the Localization Thread\n");
    }
    //
    // Get the Aria map from the localization task .
    // (Alternatively use a ArMap* made separately)
    //
    advancedptr->myMap = advancedptr->myLocaTask->getAriaMap();
    //
    // Set up the path plan structure.
    // Cannot precede localization task setup.
    //
    if(!advancedptr->initializePathPlanningTask()){
      printf("Cannot set up for path planning task thread.\n");
      exit(1);
    }else{
      printf("Started Path Planning Thread\n");
    }
    //
    // Functors to be used by the Arnl library if the threads
    // signal failure or success.
    //
    advancedptr->myLocaTask->addFailedLocalizationCB(&failed);
    advancedptr->myPathPlanningTask->addGoalDoneCB(&goal_done);
    advancedptr->myPathPlanningTask->addGoalFailedCB(&goal_failed);

  }else{
    printf(USAGE);
    exit(1);
  }

  //
  // Get user input.
  //
  interact();

}
