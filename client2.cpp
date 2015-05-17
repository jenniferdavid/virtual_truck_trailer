#include "Aria.h"
#include "ArNetworking.h"
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <iomanip>
#include <fstream>
#include <string>

#define PI 3.14
using namespace std;
using std::ofstream;


class OutputHandler  // Class for Receiving the Server data such as Server's Velocitiies, Position, Information.
{
public:
  OutputHandler(ArClientBase *client);
  virtual ~OutputHandler(void);
  
  /// This callback is called when an update on general robot state arrives
  void handleOutput(ArNetPacket *packet);

  /// This callback is called when an update on the battery configuration changes
 void handleBatteryInfo(ArNetPacket *packet);
  /// This is called when the physical robot information comes back
 void handlePhysicalInfo(ArNetPacket *packet);

  inline float getX(){return myX;}
  inline float getY(){return myY;}
  inline float getTh(){return myTh;}
  inline float getVel(){return myVel;}
  inline float getRotVel(){return myRotVel;}
  inline float getLeftVel(){return my1eftVel;}
  inline float getRightVel(){return myRightVel;}

protected:

  /// The results from the data update are stored in these variables
  //@{

  float myX;
  float myY;
  float myTh;
  float myVel;
  float myRotVel;
  float myVoltage;
  float my1eftVel;
  float myRightVel;
  char myStatus[256];
  char myMode[32];
  //@}
  ArClientBase *myClient;

  /** These functor objects are given to the client to receive updates when they
   * arrive from the server.
   */
  //@{
 ArFunctor1C<OutputHandler, ArNetPacket *> myHandleOutputCB;
 ArFunctor1C<OutputHandler, ArNetPacket *> myHandleBatteryInfoCB;
 ArFunctor1C<OutputHandler, ArNetPacket *> myHandlePhysicalInfoCB;
  //@}
  
  /// A header for the columns in the data printout is sometimes printed
  bool myNeedToPrintHeader;
  /// Don't print any information until we get the battery info
  bool myGotBatteryInfo;
};

OutputHandler::OutputHandler(ArClientBase *client) :
  myClient(client),
  myHandleOutputCB(this, &OutputHandler::handleOutput),
  myHandleBatteryInfoCB(this, &OutputHandler::handleBatteryInfo),
  myHandlePhysicalInfoCB(this, &OutputHandler::handlePhysicalInfo),
  myNeedToPrintHeader(false),
  myGotBatteryInfo(true)
{
  /* Add a handler for battery info, and make a single request for it  */
 myClient->addHandler("physicalInfo", &myHandlePhysicalInfoCB);
 myClient->requestOnce("physicalInfo");


  /* Add a handler for battery info, and make a single request for it  */
 myClient->addHandler("batteryInfo", &myHandleBatteryInfoCB);
 myClient->requestOnce("batteryInfo");

  /* Add a handler for general info, and request it to be called every 100 ms */
  myClient->addHandler("update", &myHandleOutputCB);
  myClient->request("update", 500); //specify the update rate
}

OutputHandler::~OutputHandler(void)
{
  /* Halt the request for data updates */
  myClient->requestStop("update");
}

void OutputHandler::handleOutput(ArNetPacket *packet)
{
  /* Extract the data from the update packet. Its format is status and
   * mode (null-terminated strings), then 6 doubles for battery voltage, 
   * x position, y position and orientation (theta) (from odometry), current
   * translational velocity, and current rotational velocity. Translation is
   * always milimeters, rotation in degrees.
   */
  memset(myStatus, 0, sizeof(myStatus));
  memset(myMode, 0, sizeof(myMode));
  packet->bufToStr(myStatus, sizeof(myStatus));
  packet->bufToStr(myMode, sizeof(myMode));
  myVoltage = ( (float) packet->bufToByte2() )/10.0;
  myX = (float) packet->bufToByte4();
  myY = (float) packet->bufToByte4();
  myTh = (float) packet->bufToByte2();
  myVel = (float) packet->bufToDouble();
  myRotVel = (float) packet->bufToDouble();
  my1eftVel = (float) packet->bufToDouble();
  myRightVel= (float) packet->bufToDouble();
/*
  if(myNeedToPrintHeader) 
  {
    printf("\n%6s|%6s|%6s|%6s|%6s|%6s|%15s|%20s|\n",
      "x","y","theta", "vel", "rotVel", "volts", "mode","status");
    fflush(stdout);
    myNeedToPrintHeader = false;
  }
  if (myGotBatteryInfo)
    printf("%6.0f|%6.0f|%6.1f|%6.1f|%6.1f|%6.1f|%15s|%20s|\r",
	   myX, myY, myTh, myVel, myRotVel, myVoltage, myMode, myStatus);
  */
  fflush(stdout);
}

void OutputHandler::handleBatteryInfo(ArNetPacket *packet)
{
  /* Get battery configuration parameters: when the robot will begin beeping and 
   * warning about low battery, and when it will automatically disconnect and
   * shutdown. */
  float lowBattery = packet->bufToDouble();
  float shutdown = packet->bufToDouble();
 /* printf("Low battery voltage: %6g       Shutdown battery voltage: %6g\n", lowBattery, shutdown);
  fflush(stdout);
 */ myNeedToPrintHeader = true;
  myGotBatteryInfo = true;
}


void OutputHandler::handlePhysicalInfo(ArNetPacket *packet)
{
  /* Get phyiscal configuration parameters: */
  char robotType[512];
  char robotSubtype[512];
  int width;
  int length2ront;
  int lengthRear;

  packet->bufToStr(robotType, sizeof(robotType));
  packet->bufToStr(robotSubtype, sizeof(robotSubtype));
  width = packet->bufToByte2();
  length2ront = packet->bufToByte2();
  lengthRear = packet->bufToByte2();

 /* printf("Type: %s Subtype: %s Width %d: Length2ront: %d LengthRear: %d\n",
	 robotType, robotSubtype, width, length2ront, lengthRear);
  */fflush(stdout);
}


/* Key handler for the escape key: shutdown all of Aria. */

//void escape(void)
//{rospack export --lang=cpp --attrib=lflags foobar

//  printf("esc pressed, shutting down aria\n");
 // Aria::shutdown();
//}

int main(int argc, char **argv)
{ 
	
/* Aria initialization: */
  Aria::init();
  ArRobot robot; // initalize the local robot
   /* Create our client object. This is the object which connects with a remote
   * server over the network, and which manages all of our communication with it
   * once connected by sending data "requests".  Requests may be sent once, or
   * may be repeated at any frequency. Requests and replies to requsets contain 
   * payload "packets", into which various data types may be packed (when making a 
   * request), and from which they may also be extracted (when handling a reply). 
   * See the InputHandler and OutputHandler classes above for
   * examples of making requests and reading/writing the data in packets.
   */
  ArClientBase client;

  /* Aria components use this to get options off the command line: */
  ArArgumentParser parser(&argc, argv);
  /* Initialize the sensors and the drive modes for local robot*/
  ArSonarDevice sonar;
  ArBumpers bumpers;
  ArIRs ir;
  ArSensorReading *sp = new ArSensorReading;
  ArSectors sectors;
  ArActionDesired Desired;
  ArActionRatioInput Ratio;
  ArActionBumpers bumpAct;
  ArClientSimpleConnector clientConnector(&parser);
  ArSimpleConnector connector(&argc, argv);
  parser.loadDefaultArguments();

  /* Check for -help, and unhandled arguments: */
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    exit(0);
  }

  
  /* Connect our client object to the remote server: */
  if (!clientConnector.connectClient(&client))
  {
    if (client.wasRejected())
      printf("Server '%s' rejected connection, exiting\n", client.getHost());
    else
      printf("Could not connect to server '%s', exiting\n", client.getHost());
    exit(1);
  } 

  printf("Connected to server.\n");

   // connect to local robot //connector

 if (!connector.parseArgs() || argc > 1)
  {
    connector.logOptions();
    exit(1);
  }
   // Add Range Devices to the Local Robot.
  robot.addRangeDevice(&sonar);
  robot.addRangeDevice(&ir);
  robot.addRangeDevice(&bumpers);


  // try to connect, if we fail exit
  if (!connector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }
 printf("Connected to robot.\n");


     
  /* Create a key handler and also tell Aria about it */
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  client.runAsync();
 
  // add the arcommands for Sonar, Soundtog, and motor enable commands.

  robot.comInt(ArCommands::SONAR,1);
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::SOUNDTOG, 1);
  
  robot.enableMotors();
  robot.runAsync(true);
  Ratio.activate(); //initializre the ratio mode
  
  
 OutputHandler outputHandler(&client);
 
     int ld = 350;//in mm
     float phid = 0; //in degrees
     double radphid=phid*(PI/180);
     float k1=1.1;//gain constant
     float k2=1.1;//gain constant
     float k3=1.1;//Th Gain Constant
     int h=212.5;//width of the robot in mm
     double xE, yE, thE;//error coordinates at each instant
     double xH, yH, thH;//hinge coordinates at each instant
     double  vF, wF, xF, yF, thF, radthF, radwF;// trailer parameters
     double  vL, wL, xL, yL, thL, radwL, radthL;// truck parameters

     //Setting initial parameters of the truck and trailer
 
   xL=outputHandler.getX();
   yL=outputHandler.getY();
   thL=outputHandler.getTh();

   cout << "\n _________________________ ";
   cout << "\n INITIAL POSE OF TRAILER";
   cout <<"\n"<<"xF="<< xF <<"\t"<<"yF="<<yF<<"\t"<<"thF="<<thF;
   robot.moveTo(ArPose(-700,0,0));
   xF = robot.getX();
   yF = robot.getY();
   thF= robot.getTh();
   cout << "\n SET POSE OF TRAILER";
   cout <<"\n"<<"xF="<< xF <<"\t"<<"yF="<<yF<<"\t"<<"thF="<<thF;

   cout << "\n SET POSE OF TRUCK";
   cout <<"\n"<<"xL="<< xL <<"\t"<<"yL="<<yL<<"\t"<<"thL="<<thL;
   cout << "\n ___________________________";
   cout << "\n";
  
     ArRobotPacket pkt1;
    pkt1.setID(ArCommands::SIM_SET_POSE);
    pkt1.uByteToBuf(0); // argument type: ignored.
    pkt1.byte4ToBuf(xF);
    pkt1.byte4ToBuf(yF);
    pkt1.byte4ToBuf(thF);
    pkt1.finalizePacket();
    robot.getDeviceConnection()->write(pkt1.getBuf(), pkt1.getLength());
  
    //Starting the run loop after initial values
 /********************************************************************************************************************************************************/
  
       //Waiting for the trailer to establish the virtual link
       printf("Waiting for the trailer to confirm virtual link........\n");
       fflush(stdout);

/******************************************************************************************************************************************************/
  
   int i=0;
   char timestamp[24];
     
   ofstream myfile1("client1.xls");
      myfile1<<"TIME"<<"\t"<<"\t"<<"xL"<<"\t"<<"yL"<<"\t"<<"thL"<<"\t"<<"xF"<<"\t"<<"yF"<<"\t"<<"thF"<<"\t"<<"vL"<<"\t"<<"wL"<<"\t"<<"vF"<<"\t"<<"wF"<<"\t"<<"xE"<<"\t"<<"yE"<<"\t"<<"thE"<<"\t"<<"xH"<<"\t"<<"yH"<<"\t"<<"thH"<<"\t"<<endl;


      while (robot.isRunning())
{
    robot.lock();
    time_t t = time(NULL);
    strftime(timestamp, 24, "%Y-%m-%d %H:%M:%S", localtime(&t));
    xL=outputHandler.getX();//in mm
    yL=outputHandler.getY();// in mm
    thL=outputHandler.getTh();// in degrees
    vL=outputHandler.getVel();// in mm/sec
    wL=outputHandler.getRotVel();// in deg/sec
    radwL = (wL * PI/180); // Conversion of Rotational velocity from deg to radians
    radthL = (thL * PI/180); // Conversion of theta from deg to radians
     
 //   if(thL<0)
 //   thL=thL+360;
    cout << "\n DATA AT EVERY INSTANT";
    cout <<"\n"<<"xL="<< xL <<"\t"<<"yL="<<yL<<"\t"<<"thL="<<thL;
    cout <<"\n"<<"vL="<< vL <<"\t"<<"wL="<<wL;
   
    xF = robot.getX();
    yF = robot.getY();
    thF= robot.getTh();
    
  //  if(thF<0)
  //  thF=thF+360;    
    radthF=(thF * PI/180);
    radwF = (wF * PI/180);
    cout <<"\n"<<"xF="<< xF <<"\t"<<"yF="<<yF<<"\t"<<"thF="<<thF;
         
    //Calculating absolute values for degrees
    float U,V,W,X,Y;
    int aa,bb;

    U=((radphid+radthL));
    V=(cos(U));
    W=(sin(U));
    X=(cos(radthF));
    Y=(sin(radthF));
// calculating hinge coordinates

    xH=(xL-(ld*cos(radthL)));
    yH=(yL-(ld*sin(radthL)));
    thH=radthL;

    cout << "\n"<<"xE ="<<xE<<"\t"<<"yE ="<<yE<<"\t"<<"thE"<<thE;

   //Calculate the error coordinates xe, ye and the

    xE=(xH-(ld*V)-xF)*X+(yH-(ld*W)-yF)*Y;
    yE=(-(xH-(ld*V)-xF)*Y+(yH-(ld*W)-yF)*X);
    thE=radthL-radthF;

    cout << "\n"<<"xE ="<<xE<<"\t"<<"yE ="<<yE<<"\t"<<"thE"<<thE;


    float U1,V1,W1,X1;
    U1=(cos(thE));
    V1=(sin(thE));
    W1=(sin(phid+thE));
    X1=(cos(phid+thE));

    //calculating velocity of the trailer

    wF=((vL*V1)-(ld*k3*thE*X1)-(k2*yE))/h;
    wF=(wF*h)/(1-((1-thE)*ld*X1));
    wF=(wF*180/PI);
    vF=(vL*U1)+(ld*wL*W1)+(k1*xE);    

    cout << "\n"<<"vF ="<<vF<<"\t"<<"wF ="<<wF;
    cout << "\n*********************************************************************";


    //setting velocity for the trailer
    //robot.lock();
    robot.setVel(vF);
    robot.setRotVel(wF);
    robot.unlock();
    ArUtil::sleep(500);

    myfile1<<timestamp<<"\t"<<xL<<"\t"<<yL<<"\t"<<thL<<"\t"<<xF<<"\t"<<yF<<"\t"<<thF<<"\t"<<vL<<"\t"<<wL<<"\t"<<vF<<"\t"<<wF<<"\t"<<xE<<"\t"<<yE<<"\t"<<thE<<"\t"<<xH<<"\t"<<yH<<"\t"<<thH<<endl;

    i++;
}
 myfile1.close();

//  clientSwitchManager.runAsync();

/*************************************************************************/  
  
	
  // now exit
  Aria::exit(0);
  return 0;
}



