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
       printf("Making the robot to run in sarc.......\n");
       fflush(stdout);

/**********************************************************************************************************************************************/
  int X,Y,Vel,Rotvel,VLL,VRL;
  float Th;
  char O,s,S,c,C;
     
 /*********************************************************************************************************************************************************/
 
  // float X=0, Y=0, Th=0,Vel=0,Rotvel=0, VLL=0,VRL=0;
  
   cout << "\n" << "X"<< X <<"\t"<<"Y"<<Y<<"\t"<<"Th"<<Th;
   robot.moveTo(ArPose(0,0,0));
   X=robot.getX();
   Y=robot.getY();
   Th=robot.getTh();
   cout << "\n" << "X"<< X <<"\t"<<"Y"<<Y<<"\t"<<"Th"<<Th;
   ArUtil::sleep(1000);
   

/******************************************************************************************************************************************************/
          
/*********************************************************************************************************************************************************/
 double transvelocity, rotvelocity;
	double turnangle, Rot=0,transVel=0;
	double dist=0,angle=0,obsangle,Vavoid,obsdist,AvoidObstacle=0;
	double Xrange[8];
	double Range[8]; double safewander =0, Th0=0,obsangleinit;
	int count =0, J=0; double L=0, K; double t0=0, t1=0; double test=0;
	obsdist = 1100;
	turnangle = 20;
	Vavoid = 150;
	int i=0;
 	char timestamp[24];
 	ofstream myfile("/home/jeni/Downloads/robotpose.xls");
 myfile<<"TIME"<<"\t"<<"\t"<<"Xpos"<<"\t"<<"YPos"<<"\t"<<"Th"<<"\t"<<"Vel"<<"\t"<<"Rot"<<"\t"<<"VLL"<<"\t"<<"VRL"<<"\t"<<endl;
 	ArUtil::sleep(3000);
 	while(robot.isRunning())
 	{	robot.lock();
		//keyHandler.checkKeys();
		time_t t = time(NULL);
		/// get the values of the server robot.
   		strftime(timestamp, 24, "%Y-%m-%d %H:%M:%S", localtime(&t));
   	X = robot.getX();
	Y= robot.getY();
	Th=robot.getTh();
	Vel = robot.getVel();
	Rot= robot.getRotVel();
	VLL = robot.getLeftVel();
	VRL= robot.getRightVel();

	dist = (robot.checkRangeDevicesCurrentPolar(-90,90, &angle)- robot.getRobotRadius());

	if (abs((int)angle)>180)
	{ angle = 0;
	}

	if (angle*-1 >360)
	{angle =0;
	}

				if (angle<0)
					obsangle = -1;
				else if (angle>0)
					obsangle = 1;
				else
					obsangle = 1;

	if (J==0)
	{


		if (dist < obsdist)
		{   t0 = t1;
		    t1 = dist;
				AvoidObstacle=1; safewander = 0;
				count = 0;
				if (angle<0)
					obsangle = -1;
				else if (angle>0)
					obsangle = 1;
				else
					obsangle = 1;

				transVel = 160;
				test = Vavoid * dist/obsdist;
				Rotvel = 5*obsangle*-1; 

		}
		else if (dist>obsdist)
		{	t1=dist;
			if ((count*500) > 6000) //500
			{
			AvoidObstacle =0; safewander = 1;
			Th == Th; Th0 = Th;

			if(L==0) // turn left
			{
				J=1;K=0;
			    obsangleinit = obsangle;
				transVel = 160; // 100% of maximum
				Rotvel =  5 ;//30 % of Maximum
				count == count;
				L=1;
			}

			else if(L==1)//turn right
			{
				J=1; K=1;
				obsangleinit = obsangle;
				transVel = 160;
				Rotvel =  -5 ;//-30
				count == count;
				}
			}
			else
			{ J=0;
			AvoidObstacle = 0; safewander =0;
			transVel = 160;
			Rotvel =0;
			count = count + 1;
			}
		}
	}
	else if (J<=20)//16
			{   safewander =1;
				J=J+1;
				if (K==0) // turn left
				{
				if (dist>obsdist)
				{
				transVel = 160;
				Rotvel = 5; //30
				count == count;
				L=1;
				}
				else
				{ transVel=160; //150
				  Rotvel=5*obsangle*-1;
				  J=0; L=1;
				}
				}
				else      // turn right
				{   safewander = 1;
					if (dist>obsdist)
					{transVel = 160;//150
				     Rotvel = -5; //3.4
				     count == count;
				     L=0;K=1;
					}
					else
						{ transVel=160;
						  Rotvel=5*obsangle*-1;
						  J=0;L=0;
						}
				}
			}
	else
		{ J=0; safewander=0; t1=dist;
		transVel = 160;
		Rotvel = 0;
		count =0;
		}


		transvelocity = transVel;
	    rotvelocity = Rotvel; // 100

	robot.setVel(transvelocity);
		robot.setRotVel(rotvelocity);
cout << "\n" << "X"<< X <<"\t"<<"Y"<<Y<<"\t"<<"Th"<<Th;
    myfile<<timestamp<<X<<"\t"<<Y<<"\t"<<Th<<"\t"<<Vel<<"\t"<<Rotvel<<"\t"<<VLL<<"\t"<<VRL<<"\t"<<endl;
	i++;
	robot.unlock();

   ArUtil::sleep(1000); // 500,200
 }
 myfile.close();
  clientSwitchManager.runAsync();

  robot.waitForRunExit();
  Aria::exit(0);
}




