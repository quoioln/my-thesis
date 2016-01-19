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
	ArInterpolation interp;
	ArTime t;
	ArPose pose;
	robot.runAsync(true);
	robot.enableMotors();

	robot.setVel(200);
	

	while(true) {
		//ArPose pose  = robot.getPose();
		t = robot.getLastPacketTime();
		int a = interp.getPose(t, &pose);
		ArLog::log(ArLog::Normal, "a = %d \tx = %f \t y = %f\n", a, pose.getX(), pose.getY());
		pose.log();
	} 
}
