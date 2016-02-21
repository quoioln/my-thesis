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
	//robot.lock();
	robot.comInt(ArCommands::ENABLE, 1);
	robot.addRangeDevice(&sonarDev);
	//robot.unlock();
//	ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);
	ArActionGoto gotoPoseAction("goto", ArPose(0, 0, 0), 200);
	ArActionAvoidFront avoidFront("avoid front");
	robot.addAction(&gotoPoseAction, 50);
	robot.addAction(&avoidFront, 60);

	robot.moveTo(ArPose(0,0,0));
	gotoPoseAction.setGoal(ArPose(3000, 0, 0));
	ArTime start; //timer
	start.setToNow();//start timer
	while (!gotoPoseAction.haveAchievedGoal()) {
		ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
	}
//	robot.disableMotors();
//	robot.clearDirectMotion();
//	robot.setVel(0);

	int t = 0;
	robot.deactivateActions();
	/*
	while(t * 30 <= 360) {
		robot.lock();
		robot.setDeltaHeading(30);
		robot.unlock();
//		while(!robot.isHeadingDone(30));
		t ++;
	}
*/
	robot.setDeltaHeading(180);


	robot.clearDirectMotion();
//	gotoPoseAction.activate();
	ArUtil::sleep(2000);

	gotoPoseAction.setGoal(ArPose(3000, -3000, 0));
	robot.lock();
	robot.setVel(200);
	robot.unlock();
	robot.clearDirectMotion();
//	gotoPoseAction.activate();
	while (!gotoPoseAction.haveAchievedGoal()) {
		robot.lock();
		ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
		robot.unlock();
	}
/*
	angel = 30;
	while(angel <= 360) {
		robot.setDeltaHeading(angel);
		angel += 30;
	}
*/
	ArUtil::sleep(2000);
	
	gotoPoseAction.setGoal(ArPose(0, -3000, 0));
	while (!gotoPoseAction.haveAchievedGoal()) {
		ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
	}
/*
	angel = 30;
	while(angel <= 360) {
		robot.setDeltaHeading(angel);
		angel += 30;
	}
*/
	ArUtil::sleep(2000);
	gotoPoseAction.setGoal(ArPose(0, 0, 0));
	while (!gotoPoseAction.haveAchievedGoal()) {
		ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", robot.getX(), robot.getY());
	}
/*
	angel = 30;
	while(angel <= 360) {
		robot.setDeltaHeading(angel);
		angel += 30;
	}
*/
	ArUtil::sleep(2000);
	Aria::shutdown();
}
