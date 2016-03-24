#include "Aria.h"
#include "ArNetworking.h"
class GotoGoal {
private:
	ArRobot* myRobot;
	ArSonarDevice* sonarDev;
	ArActionAvoidFront avoidFrontAction;
	ArActionGoto gotoGoalAction;
	ArActionStallRecover stallRecover;
	ArActionAvoidSide avoidSide;
	ArServerBase* server;
	ArServerInfoRobot* serverInfo;
//	ArServerCommands
public:
	GotoGoal(ArRobot* myRobot, ArSonarDevice* sonar, ArServerBase* server, ArServerInfoRobot* serverInfo);
	void init(int argc, char **argv);
	void stop();
	void lock();
	void unlock();
	ArPose getPose();
	void gotoGoal(ArPose pose);
	void rotate(float angle);
	void setVel(float vel);
	bool haveAchievedGoal();
	bool haveRotated();
	void enableDirectionCommand();
	void disableDirectionCommand();
	void shutdown();
	void move(int distance);
	void cancelGoal();
};
