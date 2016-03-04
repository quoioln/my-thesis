#include "Aria.h"
#include "Arnl.h"

using namespace std;

class GotoGoal {
private:
	ArRobot robot;
	ArSonarDevice sonarDev;
	ArActionAvoidFront avoidFrontAction;
	ArActionGoto gotoGoalAction;
//	ArPose* poseList;
public:
	GotoGoal();
	void init(int argc, char **argv);
	void stop();
	ArPose getPose();
//	void addAction(ArAction action, int prioty);
//	bool disableAction(ArAction action);
//	bool enbleAction(ArAction action);
//	ArPose* readPostitions(char* fileName);
	void gotoGoal(ArPose pose);
	void rotate(float angle);
	void setVel(float vel);
	bool haveAchievedGoal();
	bool haveRotated();
	void enableDirectionCommand();
	void disableDirectionCommand();
	void shutdown();
};
