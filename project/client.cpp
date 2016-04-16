/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2014 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
/* This is the ArNetworking example client. 
 * It connects to an ArNetworking server, and provides continuous 
 * information about the robot state (position, speed, server mode, etc.),
 * and, if the server supports the commands, lets you drive the robot with
 * the keyboard.  
 *
 * To see the example client in action, first run a server on the robot's 
 * onboard computer, for example, serverDemo, testServer, guiServer (from ARNL),
 * or ARAM. Make sure the robot computer is on a network, and run this
 * clientDemo with the hostname of the robot computer as an argument:
 *
 *    ./clientDemo -host myrobot
 *
 */

#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientHandlerRobotUpdate.h"
#include <iostream>
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/core/types_c.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


double checkObject = 0;
bool a = false;
//FILE fileTemp = NULL;
FILE *file = NULL;
void netGetFile(ArNetPacket *packet)
{
	cout <<"get file is called";
  int ret;
  char fileName[2048];
  ret = packet->bufToUByte2();
  packet->bufToStr(fileName, sizeof(fileName));
  if (ret != 0)
  {
    printf("Bad return %d on file %s\n", ret, fileName);
    exit(1);
  }
  if (file == NULL)
  {
    printf("Getting file %s\n", fileName);
    if ((file = ArUtil::fopen("ballDetect.jpg", "w")) == NULL)
    {
      printf("Can't open fileClientRaw.jpg to dump file into\n");
      exit(2);
    }
  }
  ArTypes::UByte4 numBytes;
  char buf[32000];
  //file should be good here, so just write into it
  numBytes = packet->bufToUByte4();
  if (numBytes == 0)
  {
    printf("Got all of file %s\n", fileName);
    fclose(file);
    a = true;
    ArUtil::sleep(100);
//    exit(0);
  }
  else
  {
    printf("Got %d bytes of file %s\n", numBytes, fileName);
    packet->bufToData(buf, numBytes);
    fwrite(buf, 1, numBytes, file);
  }
  //Ar
}

void recieveData(ArNetPacket* packet) {
	checkObject = packet->bufToDouble();
}
void enable(ArNetPacket* packet) {
}

int main(int argc, char **argv)
{
  Aria::init();
  ArClientBase client;
  ArArgumentParser parser(&argc, argv);

  /* This will be used to connect our client to the server. 
   * It will get the hostname from the -host command line argument: */
  ArClientSimpleConnector clientConnector(&parser);

  parser.loadDefaultArguments();

  /* Check for -host, -help, ARIA arguments, and unhandled arguments: */
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(0);
  }

  
  /* Connect our client object to the remote server: */
  if (!clientConnector.connectClient(&client))
  {
    if (client.wasRejected())
      printf("Server '%s' rejected connection, exiting\n", client.getHost());
    else
      printf("Could not connect to server '%s', exiting\n", client.getHost());
    Aria::exit(1);
  } 

  printf("Connected to server.\n");
  client.setRobotName(client.getHost()); // include server name in log messages
  client.runAsync();
  ArNetPacket request;
//  request.
//  client.requestOnceByCommand(ArCommands::ENABLE, )


  ArClientHandlerRobotUpdate updates(&client);
//  client.requestOnce("enableMotor");

  ArGlobalFunctor1<ArNetPacket*> enableCB(&enable);
  ArGlobalFunctor1<ArNetPacket *> getFileCB(&netGetFile);
  ArGlobalFunctor1<ArNetPacket*> recieveDataCB(&recieveData);

  client.addHandler("requestEnableMotor", &enableCB);
  client.addHandler("handleCheckObjectData", &recieveDataCB);
  //client.addHandler("getFile", &getFileCB);

  client.requestOnce("requestEnableMotor");
  client.request("handleCheckObjectData", 10);
  updates.requestUpdates();
  if (checkObject)
	client.requestOnceWithString("getFile", "./image/ball.jpg");
  ArPose pose;
  namedWindow("image", 0);
  while (client.getRunningWithLock())
  {

	  //client.requestOnce("sendData");

	  if (checkObject) {
	//	  if (!a) {
			  cout <<"OK"<<endl;
		//	  client.requestOnceWithString("getFile", "./image/ball.jpg");
			  client.addHandler("getFile", &getFileCB);
//			  client.remHandler("getFile", &getFileCB);
		  //}	else {
			  //client.remHandler("getFile", &getFileCB);
		  //}
		  Mat image;
		  image = imread("./image/ball.jpg", CV_LOAD_IMAGE_COLOR);
		  
		  imshow("image", image);
		  char c = (char)waitKey(10);
		  if( c == 27 )
			break;
	  }

    ArUtil::sleep(200);
  }

//  client.requestStop("getFile");

  cout <<"Da tim thay qua bong o vi tri pose("<<pose.getX()<<", "<<pose.getY()<<")"<<endl;

  cout <<"Vi tri pose("<<pose.getX()<<", "<<pose.getY()<<")"<<endl;

  /* The client stopped running, due to disconnection from the server, general
   * Aria shutdown, or some other reason. */
  client.disconnect();
  Aria::exit(0);
  return 0;
}
