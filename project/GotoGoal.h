#include "Aria.h"

class GotoGoal {
private:
	ArRobot* myRobot;
	ArSonarDevice* sonarDev;
	ArActionAvoidFront avoidFrontAction;
	ArActionGoto gotoGoalAction;
public:
	GotoGoal(ArRobot* myRobot, ArSonarDevice* sonar);
	void init(int argc, char **argv);
	void stop();
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
