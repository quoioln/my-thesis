#include "Aria.h"
using namespace std;
int main (int argc, char** argv) {
	
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

	robot.runAsync(true);
	robot.enableMotors();
	robot.lock();
	robot.setVel(200);
	robot.unlock();

	while(true) {
	//while (robot.blockingConnect()){
		robot.lock();
		ArPose pose  = robot.getPose();
		//pose.setX(100);
		//robot.moveTo(pose);
		//t = robot.getLastOdometryTime();
		//int a = interp.getPose(t, &pose);
		ArLog::log(ArLog::Normal, "x = %f \t y = %f\n", pose.getX(), pose.getY());
		robot.unlock();
	} 
}
