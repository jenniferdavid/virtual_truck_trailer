/* Example code for formation controller of Truck-Trailer.
 *
 * Copyright (C) 2014 Jennifer David. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <iomanip>
#include <fstream>
#include <string>
#include "Aria.h"
#include "ArNetworking.h"
#define PI 3.14

using namespace std;
using std::ofstream;

int main(int argc, char *argv[])
{
   Aria::init();

  // set up our parser
  ArArgumentParser parser(&argc, argv);

  // load the default arguments
  parser.loadDefaultArguments();

  // robot
  ArRobot robot;

  // set up our simple connector
  ArRobotConnector robotConnector(&parser, &robot);

  // add a gyro, it'll see if it should attach to the robot or not
  ArAnalogGyro gyro(&robot);

  // set up the robot for connecting
  if (!robotConnector.connectRobot())
  {
    printf("Could not connect to robot... exiting\n");
    Aria::exit(1);
  }

  // our base server object
  ArServerBase server;
  ArServerSimpleOpener simpleOpener(&parser);
  ArClientSwitchManager clientSwitchManager(&server, &parser);

  // parse the command line... fail and print the help if the parsing fails
  // or if the help was requested
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  // Set up where we'll look for files such as user/password
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(),
			 "ArNetworking/examples");

  // first open the server up
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    if (simpleOpener.wasUserFileBad())
      printf("Bad user/password/permissions file\n");
    else
      printf("Could not open server port\n");
    exit(1);
  }

  // Range devices:

  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);

  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);

  // attach services to the server
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoDrawings drawings(&server);

  // modes for controlling robot movement
  ArServerModeStop modeStop(&server, &robot);
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);
  ArServerModeWander modeWander(&server, &robot);
  modeStop.addAsDefaultMode();
  modeStop.activate();

  // set up the simple commands
  ArServerHandlerCommands commands(&server);
  ArServerSimpleComUC uCCommands(&commands, &robot);  // send commands directly to microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // control debug logging
  ArServerSimpleComGyro gyroCommands(&commands, &robot, &gyro); // configure gyro
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot); // control more debug logging
  ArServerSimpleServerCommands serverCommands(&commands, &server); // control ArNetworking debug logging
  ArServerSimpleLogRobotDebugPackets logRobotDebugPackets(&commands, &robot, ".");  // debugging tool

  // This is an older drive mode. ArServerModeDrive is newer and generally performs better,
  // but you can use this for old clients if neccesary.
  //ArServerModeDrive modeDrive(&server, &robot);
  //modeDrive.addControlCommands(&commands); // configure the drive modes (e.g. enable/disable safe drive)

  ArServerHandlerConfig serverHandlerConfig(&server, Aria::getConfig()); // make a config handler
  ArLog::addToConfig(Aria::getConfig()); // let people configure logging

  // You can use this class to send a set of arbitrary strings
  // for MobileEyes to display, this is just a small example
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10,
	  new ArConstRetFunctorC<int, ArRobot>(&robot,
					       &ArRobot::getMotorPacCount));
  /*
  Aria::getInfoGroup()->addStringInt(
	  "Laser Packet Count", 10,
	  new ArRetFunctorC<int, ArSick>(&sick,
					 &ArSick::getSickPacCount));
  */

  // start the robot running, true means that if we lose connection the run thread stops
  robot.enableMotors();
  robot.runAsync(true);

  drawings.addRobotsRangeDevices(&robot);

  // log whatever we wanted to before the runAsync
  simpleOpener.checkAndLog();
  // now let it spin off in its own thread
  server.runAsync();
   ArUtil::sleep(5000);

   ArLog::log(ArLog::Normal, "Server now running on port %d. Press Control-C to exit.", server.getTcpPort());
/********************************************************************************************************************************************************/
   char M, T, t, P, p;
    int X2,Y2,X1,Y1;
    float theta1,theta2;
    ArUtil::sleep(1000);

       fflush(stdout);
       printf("Enter the mode required: Truck-trailer -t/T or Parallel-Parking -p/P \n");
       fflush(stdout);
       fflush(stdin);

       scanf ("%c",&M);
       fflush(stdout);
       fflush(stdin);
       printf("Selected mode is %c \n",M);
       fflush(stdout);

       ofstream modefile("/home/jeni/Downloads/mode.txt");
        modefile<<M<<endl;

       if ((M=='t') || (M=='T'))

    	{
    	   	 ofstream truckfile("/home/jeni/Downloads/reqdpose.txt");
             X1=-1000;
             Y1=0;
             theta1=0;
             truckfile<<X1<<"\n"<<Y1<<"\n"<<theta1<<endl;
       }

       else if ((M=='p') || (M=='P'))

       {
    	   	 ofstream truckfile("/home/jeni/Downloads/reqdpose.txt");
    	   X1=0;
      	    Y1=500;
      	    theta1=0;
      	    truckfile<<X1<<"\n"<<Y1<<"\n"<<theta1<<endl;
       }
/***********************************************************************************************************************************************************/
       //Waiting for the trailer to establish the virtual link
       printf("Waiting for the trailer to confirm virtual link........\n");
       fflush(stdout);










/**********************************************************************************************************************************************/
  int X,Y,Vel,Rotvel,VLL,VRL;
  float Th;
  char O,s,S,c,C;
     
 /*********************************************************************************************************************************************************/
 
       fflush(stdout);
     printf("\n Enter the type of truck control movement: \n c/C for circle \n l/L for straightline\n");
       fflush(stdout);
       fflush(stdin);

       scanf (" %c",&O);
       fflush(stdout);
       fflush(stdin);
       printf("Selected movement is %c \n",O);
       fflush(stdout);

     ofstream infile("/home/jeni/Downloads/clientcondition.txt");
     infile<<"yes"<<endl;

  // float X=0, Y=0, Th=0,Vel=0,Rotvel=0, VLL=0,VRL=0;
  
   cout << "\n" << "X"<< X <<"\t"<<"Y"<<Y<<"\t"<<"Th"<<Th;
   robot.moveTo(ArPose(0,0,0));
   X=robot.getX();
   Y=robot.getY();
   Th=robot.getTh();
   cout << "\n" << "X"<< X <<"\t"<<"Y"<<Y<<"\t"<<"Th"<<Th;
   ArUtil::sleep(1000);
   

/******************************************************************************************************************************************************/
   if ((O=='l') || (O=='L'))
   {
   	int vel;
   	fflush(stdout);
   	printf("Enter the speed of the robot in mm/sec", vel);//mm
   	fflush(stdout);
   	fflush(stdin);
   	scanf ("%d",&vel);
   	fflush(stdout);
   	robot.lock();
   	robot.setVel(vel);
   	robot.unlock();
   }


  else if ((O=='c') || (O=='C'))
   {

   	int r;
  	float Vel=-100;//mm/sec
   	fflush(stdout);
   	printf("Enter the radius of the circle", r);//mm
   	fflush(stdout);
   	fflush(stdin);
   	scanf ("%d",&r);
   	fflush(stdout);
   	Rotvel=(Vel/r);//degrees
   	printf("rotation velocity is %f \n", Rotvel);
   	fflush(stdout);
   	printf("Rotating with a velocity of %d and radius %f \n", Vel, r);
   	fflush(stdout);

   	robot.lock();
   	robot.setVel(400);
   	robot.setRotVel(Rotvel);
   	robot.unlock();
   }

/******************************************************************************************************************************************************/
   int i=0;
   char timestamp[24];
   ofstream myfile("robotpose.xls");

   myfile<<"TIME"<<"\t"<<"\t"<<"Xpos"<<"\t"<<"YPos"<<"\t"<<"Th"<<"\t"<<"Vel"<<"\t"<<"Rot"<<"\t"<<"VLL"<<"\t"<<"VRL"<<"\t"<<endl;
   ArUtil::sleep(3000);
   while(robot.isRunning())
   {	robot.lock();
 	time_t t = time(NULL);
    strftime(timestamp, 24, "%Y-%m-%d %H:%M:%S", localtime(&t));

    X = robot.getX();
    Y= robot.getY();
    Th=robot.getTh();
    Vel = robot.getVel();
    Rotvel= robot.getRotVel();
    VLL = robot.getLeftVel();
    VRL= robot.getRightVel();
   cout << "\n" << "X"<< X <<"\t"<<"Y"<<Y<<"\t"<<"Th"<<Th;
    myfile<<timestamp<<X<<"\t"<<Y<<"\t"<<Th<<"\t"<<Vel<<"\t"<<Rotvel<<"\t"<<VLL<<"\t"<<VRL<<"\t"<<endl;
 	i++;
 	robot.unlock();

     ArUtil::sleep(500);
   }
   myfile.close();


  clientSwitchManager.runAsync();

  robot.waitForRunExit();
  Aria::exit(0);
}




