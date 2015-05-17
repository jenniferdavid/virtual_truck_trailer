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
                                                    
   

     float aa,bb,cc,dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq,rr,ss,tt,uu,vv,ww,xx;
     float l12d = 5000;//in mm
     //float alpha12d = 180; //in degrees
     float radalpha12d=PI;
     
     float l12; //in mm
     float alpha12; // in degrees
     float radalpha12;

     float k1=1;//gain constant
     float k2=1;//gain constant
     //float h=212.5;//width of the robot in mm
     float d = 3000;//in mm


     float  v2, w2, x2, y2, th2;// trailer parameters
     float  v1, w1, x1, y1, th1;// truck parameters
     float radth1,radth2,radw1,radw2;
     float ztilda[2][1];
     float gamma1, zdotd[2][1];
     float detb;
     //char n;

     float invb[2][2];
     float transb[2][2];
     float z[2][1];
     float f[2][1];
     float b[2][2];
     float u[2][1];
     float K[2][2];
     float P[2][1];
     float Q[2][1];
     float R[2][2];

    //Setting initial parameters of the truck and trailer
   //ArUtil::sleep(2000);
   x1=outputHandler.getX();
   y1=outputHandler.getY();
   th1=outputHandler.getTh();

   cout << "\n INITIAL POSE OF TRAILER";
   cout << "\n ------------------------";
   cout <<"\n"<<"x2="<< x2 <<"\t"<<"y2="<<y2<<"\t"<<"th2="<<th2;
   robot.moveTo(ArPose(-8000,0,0));
   x2 = robot.getX();
   y2 = robot.getY();
   th2= robot.getTh();
   cout << "\n SET POSE OF TRAILER";
   cout << "\n---------------------";
   cout <<"\n"<<"x2="<< x2 <<"\t"<<"y2="<<y2<<"\t"<<"th2="<<th2;

   cout << "\n SET POSE OF TRUCK";
   cout << "\n-------------------";
   cout <<"\n"<<"x1="<< x1 <<"\t"<<"y1="<<y1<<"\t"<<"th1="<<th1;
   cout << "\n";
  
     ArRobotPacket pkt1;
    pkt1.setID(ArCommands::SIM_SET_POSE);
    pkt1.uByteToBuf(0); // argument type: ignored.
    pkt1.byte4ToBuf(x2);
    pkt1.byte4ToBuf(y2);
    pkt1.byte4ToBuf(th2);
    pkt1.finalizePacket();
    robot.getDeviceConnection()->write(pkt1.getBuf(), pkt1.getLength());
  
   //CALCULATING INITIAL L12 AND ALPHA12
   l12= 5000; //in mm calculated from geometry
   radalpha12=radalpha12d;
   //radalpha12=(int(radalpha12*100))/100;// Type-casting
   cout << "radalpha12 is" << radalpha12 << "\n" ;
// ArMath::roundInt(alpha12);
   
   //Starting the run loop after initial values
 
   int i=0;
   char timestamp[24];

   ofstream myfile1("/home/jeni/Downloads/trailerposesncontrolparameters.xls");
 myfile1<<"TIME"<<"\t"<<"\t"<<"x1"<<"\t"<<"y1"<<"\t"<<"th1"<<"\t"<<"x2"<<"\t"<<"y2"<<"\t"<<"th2"<<"\t"<<"v1"<<"\t"<<"w1"<<"\t"<<"gamma1"<<"\t"<<"l12"<<"\t"<<"alpha12"<<"\t"<<"z[1][1]"<<"\t"<<"z[2][1]"<<"\t"<<"f[1][1]"<<"\t"<<"f[2][1]"<<"\t"<<"b[1][1]"<<"\t"<<"b[1][2]"<<"\t"<<"b[2][1]"<<"\t"<<"b[2][2]"<<"\t"<<"K[1][1]"<<"\t"<<"K[1][2]"<<"\t"<<"K[2][1]"<<"K[2][2]"<<"\t"<<"ztilda[1][1]"<<"\t"<<"ztilda[1][2]"<<"\t"<<"zdotd[1][1]"<<"\t"<<"zdotd[1][2]"<<"\t"<<"detb"<<"\t"<<"invb[1][1]"<<"invb[1][2]"<<"invb[2][1]"<<"\t"<<"invb[2][2]"<<"\t"<<"newl12"<<"\t"<<"newradalpha12"<<"\t"<<"v2"<<"\t"<<"w2"<<"\t"<<"radw2"<<endl;
   //check if /home/jeni/Downloads/set.text file is empty or not
   //if yes, then run the robot
  // ArUtil::sleep(10000);

   while (robot.isRunning())
   {
    robot.lock();
    time_t t = time(NULL);
    strftime(timestamp, 24, "%Y-%m-%d %H:%M:%S", localtime(&t));
    x1=outputHandler.getX();//in mm
    y1=outputHandler.getY();// in mm
    th1=outputHandler.getTh();// in degrees
    v1=outputHandler.getVel();// in mm/sec
    w1=outputHandler.getRotVel();// in deg/sec
    cout << "\n \n ******************************************************************************************************************************";
    if (th1<0)
    th1=th1+360;
    cout << "\n Iteration No:"<<i;
    cout << "\n RUNNING POSES OF TRUCK AND TRAILER";
    cout << "\n------------------------------------";
    cout <<"\n"<<"x1="<< x1 <<"\t"<<"y1="<<y1<<"\t"<<"th1="<<th1;
    //th1dot=w1;
    x2 = robot.getX();
    y2 = robot.getY();
    th2= robot.getTh();
    if (th2<0)
    th2=th2+360;
    cout <<"\n"<<"x2="<< x2 <<"\t"<<"y2="<<y2<<"\t"<<"th2="<<th2;
    cout << "\n";
    cout << "\n";
    cout << "\n INDIVIDUAL PARAMETERS OF THE CONTROL";
    cout << "\n--------------------------------------";
    
    radth1=th1*(PI/180);
    radth2=th2*(PI/180);
    gamma1=radth1+radalpha12-radth2;
    cout << "\n"<<"gamma1 is "<<gamma1;
    
    //radgamma1=gamma1*(PI/180);
    //cout << "\n"<<"radgamma1 is "<<radgamma1;

    cout << "\n"<<"l12 is "<<l12;
    cout << "\n"<<"alpha12 is "<<alpha12;

    aa= z[1][1]=l12;
    bb= z[2][1]=radalpha12;

    cout << "\n"<<"z[1][1] is "<<aa;
    cout << "\n"<<"z[2][1] is "<<bb;
    
    radw1=w1*(PI/180);
    cc=f[1][1]=-v1*cos(radalpha12);
    dd=f[2][1]=((v1*sin(radalpha12))-(l12*radw1))/l12;

    cout << "\n"<<"f[1][1] is "<<cc;
    cout << "\n"<<"f[2][1] is "<<dd;
    
    ee=b[1][1]=cos (gamma1);
    ff=b[1][2]=d*sin(gamma1);
    gg=b[2][1]=-(sin(gamma1)/l12);
    hh=b[2][2]=(d*cos(gamma1)/l12);

    cout << "\n"<<"b[1][1] is "<<ee;
    cout << "\n"<<"b[1][2] is "<<ff;
    cout << "\n"<<"b[2][1] is "<<gg;
    cout << "\n"<<"b[2][2] is "<<hh;
    
    kk=K[1][1]=k1;
    ll=K[1][2]=0;
    mm=K[2][1]=0;
    nn=K[2][2]=k2;

    cout << "\n"<<"K[1][1] is "<<kk;
    cout << "\n"<<"K[1][2] is "<<ll;
    cout << "\n"<<"K[2][1] is "<<mm;
    cout << "\n"<<"K[2][2] is "<<nn;

    oo=ztilda[1][1] = l12 - l12d;
    pp=ztilda[2][1] = radalpha12 - radalpha12d;

    cout << "\n"<<"ztilda[1][1] is "<<oo;
    cout << "\n"<<"ztilda[2][1] is "<<pp;

    qq=zdotd[1][1]=0;
    rr=zdotd[2][1]=0;

    cout << "\n"<<"zdotd[1][1] is "<<qq;
    cout << "\n"<<"zdotd[2][1] is "<<rr;

    //calculating inverse of matrix b

    //detb=1/((b[1][1]*b[2][2])-(b[1][2]*b[2][1]));
    detb=d/l12;
    cout << "\n"<<"detb is "<<detb;

    invb[1][1]=b[2][2]/detb;
    invb[1][2]=(-1*b[1][2])/detb;
    invb[2][1]=(-1*b[2][1])/detb;
    invb[2][2]=b[1][1]/detb;

    cout << "\n"<<"invb[1][1] is "<<invb[1][1];
    cout << "\n"<<"invb[1][2] is "<<invb[1][2];
    cout << "\n"<<"invb[2][1] is "<<invb[2][1]; 
    cout << "\n"<<"invb[2][2] is "<<invb[2][2];

    //calculating u from the values

    //multiplying k*ztilda
    P[1][1] =kk*oo + ll*pp;
    P[2][1] = mm*oo + nn*pp;

    Q[1][1]=-(P[1][1]+f[1][1]);
    Q[2][1]=-(P[2][1]+f[2][1]);

    R[1][1]=(invb[1][1]*Q[1][1])+(invb[1][2]*Q[2][1]);
    R[2][1]=(invb[2][1]*Q[1][1])+(invb[2][2]*Q[2][1]);

    v2=R[1][1];
    radw2=R[2][1];

    u[1][1]=v2;
    u[2][1]=radw2;

    cout << "\n"<<"u is v2 in mm/sec and w2 in rad/sec: ";
    cout << "\n"<<"v2 = "<<v2<<"\t"<<"radw2 = "<<radw2;
    w2=radw2*(180/PI);
    cout << "\n"<<"w2 in deg/sec = "<<w2;
   
    robot.clearDirectMotion();

    //setting velocity for the trailer
    //robot.lock();
    robot.setVel(v2);
    robot.setRotVel(w2);
    robot.unlock();
    ArUtil::sleep(500);
   
    cout<<"\n End of iteration no "<< i;
    cout<<"\n *******************************************************************************************************************";
    //radth2=(th2 * PI/180);
    //radth1=(th1 * PI/180);
    cout << "\n  \n \n -------------------------------------------------------------";
    cout << "\n Parameters for new l12 and alpha12";
    cout << "\n"<<"x2 is "<< x2 << "    y2 is "<< y2 <<"    th2 in rad is" <<radth2; 
    cout << "\n"<<"x1 is "<<x1 <<"    y1 is "<< y1 << "    th1 in rad is" << radth1;
   
    l12=sqrt(((x2+d*cos(th2))-x1)*((x2+d*(cos(th2))-x1))+(((y2+d*sin(th2))-y1)*((y2+d*sin(th2))-y1)));
    radalpha12=(atan2((y1-y2-d*sin(th2)),(x1-x2+d*cos(th2))));
    radalpha12=radalpha12-radth1+PI;
    //ArMath::roundInt(alpha12);
    //radalpha12=(int(radalpha12*100))/100;
    
    cout << "\n"<<"new l12 = "<<l12<<"\t"<<"new radalpha12 = "<<radalpha12;
    cout << "\n-------------------------------------------------------------";
myfile1<<timestamp<<"\t"<<"\t"<<x1<<"\t"<<y1<<"\t"<<th1<<"\t"<<x2<<"\t"<<y2<<"\t"<<th2<<"\t"<<v1<<"\t"<<w1<<"\t"<<gamma1<<"\t"<<l12<<"\t"<<alpha12<<"\t"<<z[1][1]<<"\t"<<z[2][1]<<"\t"<<f[1][1]<<"\t"<<f[2][1]<<"\t"<<b[1][1]<<"\t"<<b[1][2]<<"\t"<<b[2][1]<<"\t"<<b[2][2]<<"\t"<<K[1][1]<<"\t"<<K[1][2]<<"\t"<<K[2][1]<<K[2][2]<<"\t"<<ztilda[1][1]<<"\t"<<ztilda[1][2]<<"\t"<<zdotd[1][1]<<"\t"<<zdotd[1][2]<<"\t"<<detb<<"\t"<<invb[1][1]<<invb[1][2]<<invb[2][1]<<"\t"<<invb[2][2]<<"\t"<<l12<<"\t"<<radalpha12<<"\t"<<v2<<"\t"<<w2<<"\t"<<radw2<<endl;
    i++;
}
 myfile1.close();
 
  Aria::shutdown();
  return 0;
}
