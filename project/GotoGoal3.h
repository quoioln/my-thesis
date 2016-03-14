#include "Aria.h"
#include "ArNetworking.h"
class GotoGoal {
private:
	ArRobot* myRobot;
	ArSonarDevice* sonarDev;
	ArActionAvoidFront avoidFrontAction;
	ArActionGoto gotoGoalAction;
	ArServerBase* server;
	ArServerInfoRobot* serverInfo;
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
};
