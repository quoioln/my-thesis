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
	//robot.lock();
	robot.setVel(200);
	
	//robot.unlock();
	//ArPose pose1  = robot.getEncoderPose();
	//ArPose pose2 = pose1;
	//ArPose pose2 = ArPose(10000, 10000, 0);
	//robot.moveTo(pose2, pose1, true);
	//robot.setVel(0);
	//if (robot.isMoveDone())
		//printf ("ok");
	//ArLog::log(ArLog::Normal, "x = %.2f \t y = %.2f\n", robot.getX(), robot.getY());
	//bool check = true;
	//double list1 = {0, 0, 200, 200, 800, 1200, 1600, 1600, 1800, 2800, 2800, 2400, 1600, 1400, 1400, 0};
	//double list2 = {0, 400, 800, 1200, 1600, 1600, 1600, 2000, 2400, 2800, 2000, 1600, 1200, 800, 0, 0};
	//ArPose pose  = robot.getPose();
	//pose.setX(100);
	//pose.setY(100);
	//robot.setEncoderPose(pose);
	while(true) {
		ArPose pose  = robot.getRawEncoderPose();
		//double a = 9;
		
		ArLog::log(ArLog::Normal, "x = %f \t y = %f\n", pose.getX(), pose.getY());
		//ArLog::log(ArLog::Normal, "a = %f\n", a);
		//ArUtil:sleep(100);
		
	} 
	
	//robot.lock();
	
	//robot.unlock();		
}
