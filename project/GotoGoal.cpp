#include "GotoGoal.h"
#include<iostream>
#include<fstream>
#define ARRAY_SIZE(array)(sizeof(array[0])/sizeof(array))

using namespace std;


GotoGoal::GotoGoal(){
};
void GotoGoal::init(int argc, char **argv){
	Aria::init();
//	ArArgumentParser parser(&argc, argv);
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	if (!robotConnector.connectRobot()) {
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if(parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
//			return 1;
		}
	}
	robot.runAsync(true);
	robot.enableMotors();
//	robot.moveTo(ArPose(0,0,0));
	robot.comInt(ArCommands::ENABLE, 1);
	robot.addRangeDevice(&sonarDev);
	gotoGoalAction = ArActionGoto("goto", ArPose(0, 0, 0), 200);
	avoidFrontAction = ArActionAvoidFront("avoid front");
//	goStraightAction = ArActionGotoStraight("go straight", ArPose(0, 0, 0), 200);
	robot.addAction(&gotoGoalAction, 50);
	robot.addAction(&avoidFrontAction, 60);
//	robot.addAction(&goStraightAction, 40);
//	goStraightAction.deactivate();
};
void GotoGoal::stop(){
	robot.stop();
	robot.setVel(0);
};
/*
bool GotoGoal::disableAction(ArAction action){
	bool checked = false;
	return checked;
};
bool GotoGoal::enbleAction(ArAction action){
	bool checked = false;
	return checked;
};
*/
void GotoGoal::gotoGoal(ArPose pose){

	if (!gotoGoalAction.isActive()) {
		ArLog::log(ArLog::Normal, "action goto goal is deactive");
		return;
	}
	if (!avoidFrontAction.isActive()) {
		ArLog::log(ArLog::Normal, "action avoid front is deactive. Robot is unsafe");
		return;
	}

	gotoGoalAction.setGoal(pose);
};
void GotoGoal::rotate(float angle){
	robot.lock();
	robot.setDeltaHeading(angle);
	robot.unlock();
//	while(!robot.isHeadingDone());
};
void GotoGoal::setVel(float vel){
	robot.lock();
	robot.setVel(vel);
	robot.unlock();
};
bool GotoGoal::haveAchievedGoal(){
	return gotoGoalAction.haveAchievedGoal();
};
bool GotoGoal::haveRotated(){
	return robot.isHeadingDone();
};
void GotoGoal::enableDirectionCommand(){
	gotoGoalAction.deactivate();
	avoidFrontAction.deactivate();
};
void GotoGoal::disableDirectionCommand(){
	robot.clearDirectMotion();
	gotoGoalAction.activate();
	avoidFrontAction.activate();
};
ArPose GotoGoal::getPose(){
	return robot.getPose();
}
void GotoGoal::shutdown(){
	Aria::shutdown();
};
ArPose* readPostitions(char* fileName){
	ArPose* postitionList = new ArPose[1000];
	ArPose pose;
	ifstream is(fileName);
	char line[20];
	bool check = true;
	int i = 0;
	while (!is.eof()) {
		is >>line;
		cout <<"*"<<atoi(line)<<"*"<<endl;
		if (check) {

			pose.setX(atoi(line));
			check = false;
		} else {
			pose.setY(atoi(line));
			postitionList[i] = pose;
			check = true;
			i++;
		}
	}
	is.close();
	return postitionList;
}
int main(int argc, char **argv) {
	GotoGoal gotoGoal;
	gotoGoal.init(argc, argv);
	ArPose* poseList = readPostitions("positions.txt");
	int length = ARRAY_SIZE(poseList);
	for (int i = 0; i < length; i++) {
		gotoGoal.gotoGoal(poseList[i]);
		//ArLog(ArLog::Normal, "postive = %f, y = %f", poseList[i].getX(), poseList[i].getY());
		ArLog::log(ArLog::Normal, "postition x = %f, y = %f", poseList[i].getX(), poseList[i].getY());
		while (!gotoGoal.haveAchievedGoal()) {
			ArPose pose = gotoGoal.getPose();
			ArLog::log(ArLog::Normal, "x = %.2f, y = %.2f", pose.getX(), pose.getY());
			ArUtil::sleep(200);
		}
		/*
		int t = 1;
		gotoGoal.enableDirectionCommand();
		while(t * 30 <= 360) {
			gotoGoal.rotate(30);
			while(!gotoGoal.haveRotated());
			t++;
		}
		gotoGoal.disableDirectionCommand();
		*/
	}
	gotoGoal.shutdown();
}
