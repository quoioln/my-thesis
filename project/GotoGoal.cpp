#include "GotoGoal3.h"
//#include "iso"
#define ARRAY_SIZE(array)(sizeof(array[0])/sizeof(array))

using namespace std;


GotoGoal::GotoGoal(ArRobot* robot, ArSonarDevice* sonar){
	this->myRobot = robot;
	this->sonarDev = sonar;
//	robot = ArRobot();
//	sonarDev = ArSonarDevice();
}
void GotoGoal::init(int argc, char **argv){
	myRobot->runAsync(true);
	myRobot->moveTo(ArPose(0,0,0));
	myRobot->comInt(ArCommands::ENABLE, 1);
	myRobot->addRangeDevice(sonarDev);
	gotoGoalAction = ArActionGoto("goto", ArPose(0, 0, 0), 200);
	avoidFrontAction = ArActionAvoidFront("avoid front", 400, 200, 10);
	stallRecover = ArActionStallRecover("stallRecover");
	myRobot->addAction(&gotoGoalAction, 50);
	myRobot->addAction(&avoidFrontAction, 60);
	myRobot->addAction(&stallRecover, 70);

	myRobot->enableMotors();


};
void GotoGoal::stop(){
	myRobot->lock();
	myRobot->stop();
	myRobot->setVel(0);
	myRobot->unlock();
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
	myRobot->lock();
	myRobot->setDeltaHeading(angle);
	myRobot->unlock();
};
void GotoGoal::setVel(float vel){
	myRobot->lock();
	myRobot->setVel(vel);
	myRobot->unlock();
};
bool GotoGoal::haveAchievedGoal(){
	return gotoGoalAction.haveAchievedGoal();
};
bool GotoGoal::haveRotated(){
	return myRobot->isHeadingDone();
};
void GotoGoal::enableDirectionCommand(){
	gotoGoalAction.deactivate();
	avoidFrontAction.deactivate();
};
void GotoGoal::disableDirectionCommand(){
	myRobot->clearDirectMotion();
	gotoGoalAction.activate();
	avoidFrontAction.activate();
};
ArPose GotoGoal::getPose(){
	return myRobot->getPose();
}
void GotoGoal::shutdown(){
	Aria::shutdown();
};
/*
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
* */
/*
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
	gotoGoal.shutdown();
}
*/
