#include "Tracking.h"
#include "GotoGoal.h"
#define ARRAY_SIZE(array)(sizeof(array[0])/sizeof(array))
int main(int argc, char **argv) {
	Aria::init();
	ArRobot robot;
	ArSonarDevice sonar;
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	if (!robotConnector.connectRobot()) {
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if(parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	GotoGoal gotoGoal(&robot, &sonar);
	gotoGoal.init(argc, argv);

	Tracking tracking;
	tracking.init();
	tracking.loadCascade();
	tracking.trackbar("Trackbar");
	/*
	ArPose* poseList = readPostitions("positions.txt");
	int length = ARRAY_SIZE(poseList);
	for (int i = 0; i < length; i++) {
		gotoGoal.gotoGoal(poseList[i]);
		//ArLog(ArLog::Normal, "postive = %f, y = %f", poseList[i].getX(), poseList[i].getY());
		ArLog::log(ArLog::Normal, "postition x = %f, y = %f", poseList[i].getX(), poseList[i].getY());
		while (!gotoGoal.haveAchievedGoal()) {
			ArPose pose = gotoGoal.getPose();
			ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", pose.getX(), pose.getY());
			ArUtil::sleep(50);
		}

		int t = 1;
		gotoGoal.enableDirectionCommand();
		while(t * 90 <= 360) {
			gotoGoal.rotate(90);
			while(!gotoGoal.haveRotated());
			t++;
		}
		gotoGoal.disableDirectionCommand();
	}
	*/
	bool checkObject;
	gotoGoal.enableDirectionCommand();
	gotoGoal.setVel(200);
	float angle = 0;
	while(true) {
		checkObject = tracking.detect();
		if (checkObject){
			if(tracking.trackObject()) {
				angle =  tracking.determindRotate();
				gotoGoal.rotate(angle);
				while(!gotoGoal.haveRotated());
			} else checkObject = false;
		}
	}
	gotoGoal.shutdown();
}
