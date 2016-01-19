#include "Aria.h"
#include "AriaUtil.h"
//#include "Datalogger.h"
#include "RobotPosDir.h"
#include "ArRobot.h"
#include <fstream>
#include <iostream>
bool RotateAndPush(ArRobot *robot);
bool CentrateOnPallet(ArRobot *robot);
bool PushPallet(ArRobot *robot);
bool straight(ArRobot *robot, double degree);
double WaitForPallet(ArRobot *robot);
void ResetCoord(ArRobot *robot);
void ComeBack(ArRobot *robot);
ArRobot *myRobot;
using namespace std;
int main(int argc, char** argv)
{
//Nuestrooooooooooooooooooooooooooo
int Done=1;
//_______________+++++++++++++++++++++++++++++++++
// mandatory init
Aria::init();// set up our parser
ArArgumentParser parser(&argc, argv);
// set up our simple connector
ArSimpleConnector simpleConnector(&parser);
// robot
ArRobot robot;
// sonar, must be added to the robot, for teleop and wander
ArSonarDevice sonarDev;
///////////////////////////////////////////////////////////////////////
///////////////////
RobotPosDir PDir(&robot);
///////////////////////////////////////////////////////////////////////
///////////////////
// load the default arguments
parser.loadDefaultArguments();
// parse the command line... fail and print the help if the parsing
fails
// or if the help was requested
if (!simpleConnector.parseArgs() || !parser.checkHelpAndWarnUnparsed())
{
simpleConnector.logOptions();
exit(1);
}
// a key handler so we can do our key handling
ArKeyHandler keyHandler;
// let the global aria stuff know about it
Aria::setKeyHandler(&keyHandler);
// toss it on the robot
robot.attachKeyHandler(&keyHandler);
printf("You may press escape to exit\n");
// add the sonar to the robot
robot.addRangeDevice(&sonarDev);
// set up the robot for connecting
if (!simpleConnector.connectRobot(&robot))
{
printf("Could not connect to robot... exiting\n");
Aria::exit(1);
}
robot.runAsync(true);
// turn on the motors
robot.comInt(ArCommands::ENABLE, 1);
ofstream output;
output.open("Position.txt");
//ofstream outputY;
//outputY.open("YDist.txt");
//Datalogger DATA(&robot);
double degree= WaitForPallet(&robot);
ResetCoord(&robot);
while(robot.isConnected())
{
	/**********ADD BEHAVIORAL OR CONTROL WALL FOLLOWING CODE
HERE*************/
ArActionInput input("input");
ArActionTurn turn("turn", 200,100,60);
robot.addAction(&input,50);
robot.lock();
robot.move(20);
robot.unlock();
output << robot.getPose().getX()<< " " <<
robot.getPose().getY()<< "\n";
for (int num=2; num < 4; num++)
{
double dist=robot.getSonarRange(num);
if (dist<300)
{
myRobot->getPose().setPose(0,0,0);
while (Done==1)
{
Done= PDir.followWall();
}
}
//OUr CODE ++++++++++++++++++++++++
//Calling function to rotate and push the robot
if (Done==0)
{
straight(&robot, degree);
CentrateOnPallet(&robot);
RotateAndPush(&robot);
PushPallet(&robot);
ComeBack(&robot);
}
//++++++++++++++++++++++++++
}
}//while
output.close();
//outputY.close();
robot.waitForRunExit();
// now exit
Aria::exit(0);
}
bool RotateAndPush(ArRobot *robot)
{
myRobot= robot;
char Yes= 'y';
cout << "Rotating and pusshing pallet\n";
while (myRobot->getSonarRange(2)-myRobot->getSonarRange(3) > 15
|| (myRobot->getSonarRange(3)>500 && myRobot->getSonarRange(1)>500) )
{
myRobot->setDeltaHeading(-5);
}
return 0;
}
bool CentrateOnPallet(ArRobot *robot)
{
myRobot= robot;
while(myRobot->getSonarRange(5)<1000)
{
myRobot->lock();
myRobot->setVel(5);
myRobot->move(5);
myRobot->unlock();
}
myRobot->lock();
myRobot->setVel(50);
myRobot->move(-165);
myRobot->unlock();
return 0;
}
bool straight(ArRobot *robot, double degree)
{
myRobot=robot;
cout << "Move straigh to pallet\n";
cout << "Degrees:
" << myRobot->getTh() << endl;
if (degree==0)
{
while(myRobot->getTh()<86 || myRobot->getTh()>95)
{
cout << "Move straigh to pallet\n";
cout << "Degrees:
" << myRobot->getTh() << endl;
myRobot->setDeltaHeading(-5);
}
}
else if (degree==-90)
{
while(myRobot->getTh()<0|| myRobot->getTh()>5)
{
cout << "Move straigh to pallet\n";
cout << "Degrees:
" << myRobot->getTh() << endl;
myRobot->setDeltaHeading(-5);
}
}
lse if (degree==90)
{
while(myRobot->getTh()<175 || myRobot->getTh()>179)
{
cout << "Move straigh to pallet\n";
cout << "Degrees:
" << myRobot->getTh() << endl;
myRobot->setDeltaHeading(-5);
}
}
return 0;
}
bool PushPallet(ArRobot *robot)
{
myRobot= robot;
int count3=0;
ofstream Encoder;
Encoder.open("Encoder.txt", ios::app);
Encoder << "Pushing pallet readings \n";
Encoder << myRobot->getRightEncoder()<< " " << myRobot-
>getLeftEncoder()<< "
"<<myRobot->getTh()<<endl;
Encoder.close;
while(count3<2000)
{
cout<< "MOVING TO PUSH PALLET ________________________-----------
\n";
myRobot->lock();
myRobot->setVel(5);
myRobot->move(4000);
myRobot->unlock();
count3++;
Encoder.open("Encoder.txt", ios::app);
Encoder << myRobot->getRightEncoder()<< " " << myRobot->getLeftEncoder()<<""<<myRobot->getTh()<< endl;
Encoder.close;
}
return 0;
}
