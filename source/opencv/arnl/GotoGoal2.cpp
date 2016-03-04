#include<iostream>
#include<fstream>
#include "Aria.h"
#include "Arnl.h"
#include "ArSonarLocalizationTask.h"
#include "ArSystemStatus.h"
#define ARRAY_SIZE(array)(sizeof(array[0])/sizeof(array))

using namespace std;

class GotoGoal {
private:
	ArRobot robot;
	ArSonarDevice sonarDev;
	ArActionAvoidFront avoidFront;
	ArActionGoto gotoGoal;
	ArActionGotoStraight goStraight;
//	ArPose* poseList;
public:
	GotoGoal();
	void init();
	void stop();
//	void addAction(ArAction action, int prioty);
	bool disableAction(ArAction action);
	bool enbleAction(ArAction action);
//	ArPose* readPostitions(char* fileName);
	void gotoGoal(ArPose pose);
	void rotate(float angle);
	bool haveAchievedGoal(ArPose pose);
	bool haveRotated(ArPose pose);
	void enableDirectionCommand();
	void disableDirectionCommand();
	void shutdown();
};
