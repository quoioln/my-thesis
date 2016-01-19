/*
 * trackingobject.cpp
 *
 *  Created on: Nov 23, 2014
 *     Authors: Chris Arnold & Dallas Fletchall
 */
#include <iostream>
#include <string>
#include "Aria.h"
#include "Distance.hpp"
#include "PathLog.hpp"
#include "RobotActions.hpp"
#include "ArPoseList.hpp"

using namespace std;

int main( int argc, char** argv ){

   // Read all poses from the file
   ArPoseList poses("../Data/object.dat");

   Aria::init();

   ArArgumentParser parser(&argc, argv);

   parser.loadDefaultArguments();

   ArRobot robot;
   ArRobotConnector robotConnector(&parser, &robot);

   if( !robotConnector.connectRobot() ){

      if( !parser.checkHelpAndWarnUnparsed() ){
         ArLog::log(ArLog::Terse, "Could not connect to robot");
      }else{
         ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
         Aria::logOptions();
         Aria::exit(1);
      }
   }

   if( !robot.isConnected() ){
      ArLog::log(ArLog::Terse, "Internal error: connection failed");
   }

   if( !Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed() ){
      Aria::logOptions();
      Aria::exit(1);
      return 1;
   }

   ArKeyHandler keyHandler;
   Aria::setKeyHandler(&keyHandler);
   robot.attachKeyHandler(&keyHandler);
   printf("You may press escape to exit\n");

   robot.runAsync(true);

   ArUtil::sleep(500);

   robot.lock();

   robot.comInt(ArCommands::ENABLE, 1);

   robot.unlock();

   PathLog log("../wander.txt");

   ArPose pose, prev_pose = robot.getPose();
   double total_distance = 0;

   while( poses.getPose(&pose) ){


      moveRobot(&robot, pose);
      ArUtil::sleep(500);
      pose = robot.getPose();
      log.write(pose);
      total_distance += getDistance(prev_pose, pose);
      prev_pose = pose;

   }

   ArUtil::sleep(1000);
   log.close();

   ofstream output;
   output.open("../wander_dist.txt", ios::out | ios::trunc);
   output << "Wander .5 " << total_distance << endl;
   output.close();

   Aria::exit(0);
   return 0;

}
