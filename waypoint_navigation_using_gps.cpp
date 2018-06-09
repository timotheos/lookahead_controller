//***************************************************************************
//***************************************************************************
// Created by Yu Yu Lin
//            PhD Student
//            Graduate School of Science and Technology
//            Tokai University
//
// Mobile robot is steering by Look-ahead controller based on two modes,
// namely goal-aiming and obstacle-avoiding, within the waypoint-based
// navigation systen using GPS.
//***************************************************************************
//***************************************************************************

#include "Aria.h"
#include "time.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <fstream>
#include <conio.h>

Using namespace std;
#define RAD (180/3.1415926535897)
int flagAvoid = 0;

//***************************************************************************
//*********************************yuGPS*************************************
//***************************************************************************

#define NAUTICAL_MILES 3437.73
#define METERS_PER_NM 1852.0
#define RAD_TO_DEG (180/3.14159265358979)
//***** For connecting to devices through a serial port

class yuSerialConnection {
  public:
    yuSerialConnection();           /* Constructor */
    virtual ~yuSerialConnection();  /* Destructor also closes the connection */
    int Open(const char* port = NULL, int baud = 9600);
    void SetPort(const char *port = NULL);

    virtual int getStatus(void);
    virtual bool close(void);
    virtual int read(char *data, unsigned int size, unsigned int msWait = 0);
    virtual const char * getOpenMessage(int messageNumber);

    /*Sets the baud rate on the connection*/
    bool setBaud(int baud); int internalOpen(int yuBaudRate);
    /*Internal open, for use by open and openSimple*/
    enum Open {
      OPEN_COULD_NOT_OPEN_PORT = 1, /*Could not open the port*/
      OPEN_COULD_NOT_SET_UP_PORT,   /*Could not set up the port*/
      OPEN_INVALID_BAUD_RATE,       /* Baud rate is not valid*/
      OPEN_COULD_NOT_SET_BAUD       /*Baud rate valid, but could not set it*/
      OPEN_ALREADY_OPEN             /*Connection was already open*/
    };
    enum Status {
      STATUS_NEVER_OPENED = 1,      /*Never opened*/
      STATUS_OPEN,                  /*Currently open*/
      STATUS_OPEN_FAILED,           /* Tried to open, but failed*/
      STATUS_CLOSED_NORMALLY,       /*Closed by a close call*/
      STATUS_CLOSED_ERROR           /*Closed because of error*/
    };
    virtual ArTime getTimeRead(int index);
    virtual bool isTimeStamping(void);
    HANDLE myPort;

  protected:
    void buildStrMap(void);
    int rate oBaud(int rate);      /*these both return -1 for errors*/
    int baudToRate(int baud);
    void startTimeStamping(void);
    bool myTakingTimeStamps;
    ArStrMap myStrMap;
    std::string myPortName;
    int myBaudRate;
    int myStatus;
    bool myHardwareControl;
};
//*****(1) Constructor
yuSerialConnection::yuSerialConnection() {
  myPort = INVALID_HANDLE_VALUE;
  myBaudRate = 9600;
  myStatus = STATUS_NEVER_OPENED;
  myHardwareControl = false; buildStrMap();
}

//*****(2) Destructor
yuSerialConnection::~yuSerialConnection() {
  if (myPort != INVALID HANDLE_VALUE) {
    close();
  }
}

//*****(3)Building string message.
void yuSerialConnection::buildStrMap(void) {
  myStrMap[OPEN_COULD_NOT_OPEN_PORT] = "Could not open serial port.";
  myStrMap[OPEN_COULD_NOT_SET_UP_PORT] = "Could not set up serial port.";
  myStrMap[OPEN_INVALID_BAUD_RATE] = "Baud rate invalid, could not set
                                      baud on serial port.";
  myStrMap[OPEN_COULD_NOT_SET_BAUD] =  "Could not set baud rate on serial port.";
  myStrMap[OPEN_ALREADY_OPEN] = "Serial port already open.";
}

//***** (4) Giving out open message.
const char * yuSerialConnection::getOpenMessage(int messageNumber) {
   return myStrMapsmessage Number).c_str();
 }

//***** (5) Setting port.
void yuSerialConnectionsetPort(const char *port){
  if (port == NULL){
    myPortName="COM2";
    myPortName="COM4";
  }
  else { myPortName = port; }
}
//***** (6) Openning the port with the parameter value of port no. and baud rate.
int vuSerialConnection::open(const char *port, int yubaud) {
  setPort(port);
  return internalOpen(yubaud);
}

//***** (7) Internal openning the port.
int yuSerialConnection::internalOpen(int yuBaudRate) {
  DCB dcb;
  dcb.DCBlength = sizeof(dcb);

  if (myStatus == STATUS OPEN) {
    ArLog::log(ArLog::Terse, "yuSerialConnection::open: Serial port already open");
    return OPEN_ALREADY_OPEN;
  }

  myPort = CreateFile(myPortName.c_str(),
      GENERIC READ|GENERIC_WRITE,
      0,        /* exclusive access */
      NULL,     /* no security attrs */
      OPEN_EXISTING,
      0,        /* not overlapped I/O */
      NULL);

  if (myPort == INVALID_HANDLE_VALUE) {
    ArLog::log(ArLog::Terse, "yuSerialConnection::open: Could not open serial port '%s",
      myPortName.c_str());
    return OPEN_COULD_NOT_OPEN_PORT;
  }
  if (!GetCommState(myPort, &dcb)) {
    ArLog::log(ArLog::Terse, "yuSerialConnection::open: Could not get port data to set up port");
    close();
    myStatus = STATUS_OPEN_FAILED;
    return OPEN_COULD_NOT_SET_UP_PORT;
  }

  COMMTIMEOUTS commTimeouts;
  commTimeouts.ReadIntervalTimeout=MAXDWORD;
  commTimeouts.ReadTotalTimeoutMultiplier = 0;   /* No delay in reading */
  commTimeouts.ResdTotalTimeoutConstant = 100;
  commTimeouts.WriteTotalTimeoutMultiplier = (20000 / dcb.BaudRate) ? (20000 /
    dcb.BaudRate) : 1;
  commTimeouts.WriteTotalTimeoutConstant = 0;    /* No delay in writing */

  if(!SetCommTimeouts(myPort, &commTimeouts)) {
  printf("Error in InternalOpen::SetCommTimeout with error %d.\n". GetLastError());

  Secure ZeroMemory(&dcb, sizeof(dcb));
  dcb.DCBlength = sizeof(dcb);
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT:
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = 0;
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  dcb.fNull   = FALSE;
  dcb.fOutX   = FALSE;
  dcb.finx    = FALSE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;

  if ( !SetCommState(myPort, &dcb)) {
    printf ("SetCommState failed with error %d.\n", GetLastError());
    ArLog::log(ArLog::Terse, "yuSerialConnection::open: Could not set up port");
    close();
    myStatus = STATUS_OPEN_FAILED;
    return OPEN_COULD_NOT_SET_UP_PORT;
  }

  myStatus = STATUS OPEN;
  if (!setBaud(yuBaudRate)) {
    ArLog::log(ArLog::Terse, "yuSerialConnection::open: Could not set baud rate.");
    close();
    myStatus = STATUS OPEN FAILED;
    return OPEN COULD NOT_SET_BAUD;
  }

  ArLog::log(ArLog::Verbose, "yuSerialConnection::open: Successfully opened
  and configured serial port.");
  return 0;
}

//****** (8) Closing the port.
bool yuSerialConnection::close(void) {
  bool ret;
  if (myPort == INVALID HANDLE_VALUE)
  return true;
  SetCommMask(myPort, 0);                 /* disable event notification */
  EscapeCommFunction( myPort, CLRDTR );   /* drop DTR */
  /* purge any outstage any outstanding reads/writes and close device handle */
  PurgeComm( myPort, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );

  My Status = STATUS CLOSED NORMALLY;
  ret = CloseHandle( myPort );
  if (ret)
    {ArLog::log(ArLog::Verbose,"yuSerialConnection::close: Successfully
                                closed serial port.");}
  else
    {ArLog::log(ArLog::Verbose,"yuSerialConnection::close: Unsuccessfully
                                closed serial port.");}
  myPort = (HANDLE)INVALID_HANDLE VALUE;
  return ret;
}

//****** (9) Setting the baud rate for the serial connection.
bool yuSerialConnection::setBaud(int baud) {
  DCB dcb;
  myBaudRate = baud:
  if (getStatus() != STATUS_OPEN))  return true;
  if ( !GetCommState(myPort, &dcb)) {
    printf ("GetCommState failed with error %d. \n", GetLastError());
    ArLog::log(ArLog::Terse, "yuSerialConnection::setBaud: Could not get port Data.");
    return false;
   }
  dcb.BaudRate = myBaudRate;
  if ( !SetCommState(myPort, &dcb) ) {
    printf ("SetCommState failed with error %d. \n", GetLastError());
    ArLog::log(ArLog::Terse, "yuSerialConnection::setBaud: Could not set port Data.");
    return false;
  }
  return true;
}

//***** (10) Reading data from the serial port.
int yuSerialConnection::read( char *data, unsigned int size, unsigned int msWait) {
  COMSTAT stat;
  unsigned long ret;
  unsigned int numToRead;
  ArTime timeDone;

  if (myPort != INVALID_HANDLE_VALUE && myStatus == STATUS_OPEN) {
    if (msWait > 0) {
      timeDone.setToNow();
      timeDone.addMSec(msWait);
      while (timeDone.mSecTo() >=0) {
        if (!ClearCommError(myPort, &ret, &stat)) return -1;
        if (stat.cbInQue < size) ArUtil::sleep(2);
        elses break;
      }
  if (!ClearCommError(myPort, &ret, &stat)) {return -1;}
  if (stat.cbInQue == 0) {return 0;}
  if (stat.cblnQue > size) {numToRead = size;}
    else numToRead = stat.cbInQue;
  if (ReadFile(myPort, data, num ToRead,&ret, NULL)) { return (int)ret; }
  else {
    ArLog::log(ArLog::Terse, "yuSerialConnection::read: Read failed.");
    return -1;
  }
  ArLog::log(ArLog::Terse, "yuSerialConnection::read: Connection invalid.");
  return -1;
}

//***** (11) Getting the port status.
int yuSerialConnection::getStatus(void) { return myStatus; }

//***** (12) Time.
bool yuSerialConnection::isTimeStamping(void) { return false; }

//***** (13) Time.
ArTime yuSerialConSerialConnection::getTimeRead(int index) {
  ArTime now;
  now.setToNow();
  return now;
}
//************************ End of yuSerialConnect *************************

//*********** Converting degreeMinutes to decimalDegrees ******************
double degminToDecdeg(double degmin) {
  double degrees;
  double minutes = modf(degmin / (double)100.0, &degrees) * (double) 100.0;
  return degrees + (minutes / (double) 60.0);
}
//**************************** End of yuGPS *******************************
//*************************************************************************

//*************************************************************************
//************************* Obstacle avoidance ****************************
//*************************************************************************
class yuAvoidFront : public ArAction {
  public:
    yuAvoidFront(const char *name = "avoid front obstacles", double obstacle Distance = 450,
          double avoidVelocity = 200, double turnAmount = 15);
    virtual ~yuAvoidFront(){}
    virtual ArActionDesired *fire(ArActionDesired currentDesired);
    virtual ArActionDesired *getDesired(void) { return &myDesired; }

  protected:
    double myTurnAmount;
    double myObsDist;
    double myAvoidVel;
    double myTurnAmountParam;
    int myTurning;        /* 1 for turning left, 0 for not turning, -1 for turning right*/
    ArActionDesired myDesired;
    ArSectors myQuadrants;
    ArFunctorC<yuAvoidFront> myConnectCB;
};

yuAvoidFront::yuAvoidFront(const char *name, double obstacleDistance,
                          double avoidVelocity, double turnAmount) :
  ArAction(name, "Slows down and avoids obstacles in front of the robot.") {
    setNextArgument(ArArg("obstacle distance", &myObsDist, "Distance at which to turn.
      (mm)"));
      myObsDist = obstacleDistance;

    setNextArgument(ArArg("avoid speed", &myAvoidVel, "Speed at which to go
    while avoiding an obstacle. (mm/sec)"));
    myAvoidVel = avoidVelocity;

    setNextArgument(ArArg("turn amount", &myTurnAmountParam, "Degrees to turn
        relative to current heading while avoiding obstacle (deg)"));
    myTurnAmountParam = turnAmount;
    myTurning = 0;
};                                  /* No turning */

ArActionDesired *yuAvoidFront::fire(ArActionDesired currentDesired) {
  double dist, angle;
  if (currentDesired.getDeltaHeadingStrength() >= 1.0) {myTurning = 0;}

  myDesired.reset();
  dist = (myRobot->checkRangeDevicesCurrentPolar(-70, 70, &angle) - myRobot-
  >getRobotRadius());

  if (dist > myObsDist) {
    flagAvoid = 0;                                    /* no avoiding */
    if (myTurning != 0) {
      myDesired.setDeltaHeading(0);
      myTurning = 0;
      return &myDesired;
    }
    else {
      myTurning = 0;
      return NULL;
    }
  }

  flagAvoid = 1;                                      /* Avoiding */

  if (myTurning == 0) {
    if (angle <0) { myTurning = 1; }
    else { myTurning = -1; }
    myTurnAmount = myTurnAmountParam;
    myQuadrants.clear();                              /* start revolution */
  }

  myQuadrants.update(myRobot->getTh());
  if (myTurning && myQuadrants.didAll()) {
    myQuadrants.clear();
    myTurnAmount /= 2;
    if (myTurnAmount == 0) myTurnAmount = myTurnAmountParam;
  }

  myDesired.setDeltaHeading(my Turning * myTurnAmount);

  if (dist > myObsDist/2) {
    myDesired.setVel(myAvoidVel * dist / myObsDist);
  }
  else { myDesired.setVel(0); }
  return &myDesired;
}
//*************************** End of YuAvoid *******************************

//**************************************************************************
//** Look-ahead Controller in two modes: goal-aiming and obstacle-avoiding **
//**************************************************************************

fstream wayLa("yuWayPointX.txt",ios::in);
fstream WayLo("yuWayPointY.txt",ios::in);
yuAvoidFront yuAvoid("yuAvoid",300,100,15);

FILE *rx, *ry, *rt, *rrx, *rry, *rrt:

class ArLookAheadControl {
  public:
    ArLookAheadControl (ArRobot *robot, double vel, const double gain, double
      lookahead_distance, double *crtgpsx, double * crtgpsy, double *crtgpsthi,
      double *tXX, double *tYY, double *bH, int *a);
    ~ArLookAheadControl(void) {}
    void doTask(void);

  protected:
    ArRobot *myRobot;
    ArActionInput yuAction;
    ArTime myStartTime;
    ArFunctorC<ArLookAheadControl> myTaskCB:

    double y_hat_x, y hat y;
    double y_hat_x_dot, y_hat_y dot;
    double Kp, KpOfdudk;
    double u_eq1, u_eq2;                      /* new input */
    double phi[3][3];
    double det;
    double inv_phi[3][3];
    double eta_input1, eta_input2;            /* input for mobile robot */
    double robox, roboy;
    double init_ref_x, init_ref_y;
    double robophi;
    double x_r, y_r;
    double vel, Rvel;
    double y_eq1, y_eq2;


    double omega, radius;
    double wayPtX[1000], wayPtY[1000];
    double xx, yy;
    double L, Lx, Ly, alpha, l, alphaLc;      //< dist bet COM and next waypoint
    int ii, no, i;
    double dist, distX, distY, div;           //< dist bet two way points
    double myProcess Time[100];
    double delta, wx, wy;
    double rbx, rby, rbt, obsDist, beta;
    double *CrtGpsXX, *CrtGpsYY, *crtgpsThi;
    double crtX, crtY, crtAng;
    FILE *dd;
    double *TXX, *TYY, txx, tyy;
    double *BH, baseh, TrGps X, TrGpsY;
};

//*** the constuctor, note how it uses chaining to initialize the myTaskCB
ArLookAheadControl::ArLookAheadControl(ArRobot *robot, double vel,
  const double gain, double lookahead_distance, double *crtgpsx, double
  *crtgpsy, double *crtgpsthi, double *tXX, double *tYY, double *bH, int *a):

myTaskCB(this, &ArLookAheadControl::doTask) {
  myRobot = robot;
  myStartTime.setToNow();
  myRobot->addUserTask("ArLookAheadControl", 50, &myTaskCB);

  Kp = gain;
  init_ref_x = lookahead_distance:
  init_ref_y = 0.0;                             // mm

  rx = fopen("AvoidMyuRobotX.txt", "w+");       //robot's coordinates and heading angle
  ry = fopen("AvoidMyuRobotY.txt", "w+");       //while it is in goal-aiming mode
  rt = fopen("AvoidMLyuRobotThi.txt", "w+");
  rrx = fopen("GoalAimMyuRobotX.txt", "w+");
  rry = fopen("GoalAimMyuRobotY.txt", "w+");    // robot's coordinates and heading angle
  rrt = fopen("GoalAimMyuRobotY.txt", "w+");    // while it is in obstacle-avoiding mode

  Lx      = 0.0; Ly      = 0.0; L     = 0.0; alpha   = 0.0;   l = 0.0;
  ii      = 0;   no      = 0;   delta = 0.0; alphaLc = 0.0;
  y_hat_x = 400; y_hat_y = 0;   div   = vel;

  for(i=0; i<100; i++) { myProcess Time[i] = 0.0; }

  if(!wayLa || !wayLo) {
    printf(" Exit due to not having way point files \n");
    exit(2);
  }
  else {
    while(! wayLa.eof() || !wayLo.eof()) {
      wayLa>>xx;      wayPtX[ii] = xx;
      wayLo>>yy;      wayPtY[ii] = yy;
      ii++;
    }
  }

  printf(" Number of way points : %d\n\n", ii);
  dist = sqrt((wayPtX[0] * wayPtX[0]) + (wayPtY[0] * wayPtY[0]));
  myProcess Time[0] = dist/div;

  wx   = 0.0; wy   = 0.0; obsDist = 0.0; beta = 0.0;
  rbx  = 0.0; rby  = 0.0; rbt     = 0.0;
  crtX = 0.0; crtY = 0.0; crtAng  = 0.0;
  modWPX = 0.0; modWPY = 0.0;
  CrtGpsXX = crtgpsx; CrtGpsYY = crtgpsy; crtgpsThi=crtgpsthi;
  BH = bH;            TXX = tXX;           TYY = tYY;
  dd = fopen("yuCheckData.txt","w+");

//**** Main task of look-ahead controller
ArLookAheadControl::doTask(void) {

     (*myRobot).addAction(&yuAvoid, 99);
     (*myRobot).addAction(&yuAction, 50);
     time_t msec;
AA:  msec = myStartTime.mSecSince();
     double myTime = double(msec/1000) + double(msec%1000)/1000;

  if ((myTime < myProcessTime[no]) && no <ii) {
    baseH = *BH;
    robox = *TXX;                       //mm
    roboy = *TYY;
    robophi = (*crtgps Thi) - baseH;
  }

  rbx = robox;    rby = roboy;    rbt = robophi;
  // Distance between look-ahead point and COM of robot
  x r= init_ref_x;          y_r= init_ref_y;

  // Dist & angle between COM & current waypoint
  Lx      = wayPtX[no] - robox;
  Ly      = wayPtY[no] - roboy;
  L       = sqrt( (Lx * Lx) + (Ly * Ly) );
  alpha   = atan2(Ly,Lx) - robophi;   // rad
  alphaLc = atan2(Ly,Lx);             // rad

  // Angle & dist between look-ahead & current waypoint
  l = sqrt( (x_r*x_r) + (L*L) - (2 * x_r * L * cos(alpha)) );

  //*** Check Obstacle dist & change Lc -> Rb
  myRobot->lock();
  obsDist = myRobot->checkRangeDevicesCurrentPolar(-90, 90, &beta);
  myRobot->unlock();

  //**********************************
  //***** Obstacle-avoiding Mode *****
  //**********************************

  if((flagAvoid == 1) || (obsDist < 1500 )) {
    if(L < 1000) { delta= 170; }
    else         { delta = 80; }

    if(flagAvoid ==1) {
      myProcessTime[no] = myProcessTime[no] + 0.22;
      delta = 150;
    }
    printf("Obstacle-avoiding Mode \t Flag %d \n", flagAvoid);

    y_hat_x_dot = 300 * cos(alpha);
    y_hat_y dot = 300 * sin(alpha);
    y_hat_x     = 400; y_hat_y   = 0;             /* lookahead_distance */

    //Coordinates of reference point
    y_hat_x = y_hat_x + y_hat_x_dot * 0.1;        /* mm */
    y_hat_y = y_hat_y + y_hat_y_dot * 0.1;

    // Output equation for Obstacle-avoiding mode
    y_eq1 = x_r + delta * cos(alpha);
    y_eq2 = delta * sin(alpha);

    // Decoupling matrix inverse for Obstacle-avoiding mode
    inv_phi[1][1] = 1.00;
    inv_phi[1][2] = 0;
    inv_phi[2][1] = 0;
    inv_phi[2][2] = (1.00/x_r);

    fprintf(rrx, " %0.2f \n", rbx);
    fprintf(rry, " %0.2f \n", rby);
    fprintf(rrt, " %0.2f \n". ArMath::radToDeg(rbt));    /* rad */

    //New input
    u_eq1 = y_hat_x_dot + Kp*(y_hat_x - y_eq1)
    u_eq2 = y_hat_y_dot + Kp*(y_hat_y - y_eq2);

    //Control input for mobile robot
    eta_input1 = inv_phi[1][1] * u_eq1 + inv_phi[1][2] * u_eq2;  /* vel of robot */
    eta_input2 = inv_phi[2][1] * u_eq1 + inv_phi[2][2] * u_eq2;  /* vel of robophi */

    vel = eta_input1;
    Rvel = ArMath::radToDeg(eta_input2);
    myRobot->lock();
    yuAction.setVel(vel);
    yuAction.setRotVel(Rvel);
    myRobot->unlock();

  //*** Instance value while changing from Obstacle-avoiding mode to goal-aiming mode
    y_hat_x = robox + x_r;
    y_hat_y= roboy + y_r;
  }

  //****************************
  //***** Goal-aiming Mode *****
  //****************************

  else {
    printf(" Goal-aiming Mode \n");
    y_hat_x_dot = 250 * cos(alphaLc);
    y_hat_y_dot = 250 * sin(alphaLc);

    //Coordinates of reference point
    if ( L < 1000) {
      y_hat_x = robox + (0.75) * (x_r * cos(alphaLc) - y_r * sin(alphaLc));
      y_hat_y = roboy + (0.75) * (x_r * sin(alphaLc) + y_r * cos(alphaLc));
      printf(" ** ");
    }
    else {
      y_hat_x = robox + x_r * cos(alphaLc) - y_r * sin(alphaLc);
      y_hat_y = roboy + x_r * sin(alphaLc) + y_r * cos(alphaLc);
    }
    // Output equation for Goal-aiming mode
    y_eq1 = robox + x_r * cos(robophi) - y_r * sin(robophi);
    y_eq2 = roboy + x_r * sin(robophi) + y_r * cos(robophi);

    // Decoupling matrix for Goal-aiming mode
    phi[1][1] = cos(robophi);
    phi[1][2] = -x_r*sin(robophi) - y_r*cos(robophi);
    phi[2][1] = sin(robophi);
    phi[2][2] =  x_r*cos(robophi) - y_r*sin(robophi);

    // Inverse of decoupling matrix Phi
    det = phi[1][1] * phi[2][2] - phi[2][1] * phi[1][1];
    inv_pil[1][1] =  phi[2][2]/det;
    inv_phi[1][2] = -phi[1][2]/det;
    inv_phi[2][1] = -phi[2][1]/det;
    inv_phi[2172] =  phi[1][1]/det;

    fprintf(rx, " %0.2f \n", rbx);
    fprintf(ry, " %0.2f \n", rby);
    fprintf(rt, " %0.2f \n", ArMath.radToDeg(rbt));

    //New input to the mobile robot
    u_eq1 = y_hat_x_dot + Kp*(y_hat_x - y_eq1);
    u_eq2 = y_hat_y_dot + Kp*(y_hat_y - y_eq2);

    //Control input for mobile robot
    eta_input1 = inv_phi[1][1] * u_eq1 + inv_phi[1][2] * u_eq2; /*vel of robot*/
    eta input2 = inv_phi[2][1] * u_eq1 + inv_phi[2][2] * u_eq2; /*vel of robophi*/

    vel = eta_inputl;
    Rvel = ArMath::radToDeg(eta_input2):
    myRobot->lock();
    yuAction.setVel(vel);
    yuAction.setRotVel(Rvel);
    myRobot->unlock();
  }
  // ********************************************
  // ***** Checking next waypoint to attain *****
  // ********************************************

  wx = wayPtX[no];        wy = wayPtY[no);
  if((WX-320 < robox) && (robox < wx+320) && (wy-320 < roboy) && (roboy < wy+320)) {
    printf("\t***");
    no++;
    distX = wayPtX[no] - wayPtX[no-1];
    distY = wayPtY[no] - wayPtY[no-1];
    dist = sqrt((distX * distX) + (dist Y * distY));
    myProcessTime[no] = myProcessTime[no-1] + (dist/div);

    //*** Get current position through GPS
    crtX = *CrtGps XX;                              /* mm */
    crtY = *CrtGps YY;
    crtAng = ArMath::degToRad(*crtgps Thi);         /* rad */

    txx = *TXX;                                     /* ECEF Gps XY */
    tyy = *TYY;
    goto AA;
  } // End of if (my Time < myProcess Time)

  else {
    printf("Ok");
    myRobot->lock();
    yuAction.setVel(0);
    yuAction.setRotVel(0);
    myRobot->unlock();
  }
}

//********* End of Look-ahead of Look-ahead Controller in two modes **********
//****************************************************************************

//****************************************************************************
//***************************** Function Main ********************************

int main(int argc, char **argv) {
  //*** Printing out the program start time.
  time_t runtime;
  FILE *tim;
  tim = fopen("yuRunTime.txt","w+").

  if( time(&runtime) ==-1) {
    printf("Calendar time not available. \n");
    exit(1);
  }
  else fprintf(tim, "The start time is \t\t %s \n", ctime(&runtime);

  //** Robot and Sick connection.
  ArRobot robot;                    //< the robot
  ArSick sick;                      //< the laser
  ArSerialConnection robotConn;     //< the robot serial connection
  ArSerialConnection sickConn;      //< SICK serial connection
  ArTcpConnection    tcpConn;       //< Simulation tcp connection
  bool useSim;
  Aria::init();

  //** All the information for our printing out Robot and Sick data.
  double x,y;
  std::list<ArPoseWithTime *> *readings;
  std::list<ArPoseWithTime *>::iterator it;

  FILE *sickX,*sickY;
  sickX = fopen("yuSickX.txt", "w+");
  sickY = fopen("yuSickY.txt", "w+");

  //** Add the laser to the robot
  robot.addRangeDevice(&sick);

  //** See if we can get to the simulator (true is success)
  tcpConn.setPort();
  if(tcpConn.openSimple()) {
    printf("Connecting to simulator through TCP \n");
    robot.setDeviceConnection(&tcpConn);
    useSim = true;
  }
  else {
    useSim = false;
    // We could not get to the simulation, so set the port for
    // the serial connection and then set the serial connection as robot device.
    robotConn.setPort();      /< Default port for robot is COM1
    printf(" Could not connect to the simulator. Connecting to robot through serial. \n");
    robot.setDeviceConnection(&robotConn);

    //** Try to connect. if we fail exit
    if(!robot.blockingConnect()) {
      printf("Could not connect to robot... exiting\n");
      Aria::shutdown();
      return 1;
    }

    //** Start the robot running. True so that if we lose connection, the run stops.
    robot.runAsync(true);
    ArActionJoydrive joydrive;
    joydrive.setStoplfNoButtonPressed(false);
    joydrive.setThrottleParams(100, 400);

    //** Configure and setup the SICK
    sick.configureShort(useSim, ArSick::BAUD38400. ArSick::DEGREES180,
              ArSick::INCREMENT_ONE);
    sickConn.setPort(ArUtil::COM3);
    sick.setDeviceConnection(&sickConn);
    sick.runAsync();

    //** Check SICK connection
    if(!sick.blockingConnect()) {
      printf("Could not connect to SICK laser... exiting\n");
      Aria::shutdown();
      return 1;
    }
    if(sick.isConnected()) { printf("SICK Connected\n"); }
    else { printf("SICK: not connected\n"); }

    //** Turn on the motors, turn off amigobot sounds and turn on joydrive.
    robot.comInt(ArCommands::ENABLE, 1);
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.comInt(ArCommands::JOYDRIVE, 1);
    robot.addAction(&joydrive, 100);

    //************************************************************************
    //*************** Configuring the desired serial port for GPS ************
    //************************************************************************
    yuSerialConnection gpsConn;
    gpsConn.setPort("COM2");        //< for the sake of port no.

    if(gpsConn.setBaud(9600)) { printf(" Finished setting Baud rate!\n"); }
    else {
      printf(" Due to not setting the baud rate. ~-~ !! \n");
      gpsConn.close();
      exit(1);
    }

    if(gpsConn.open("COM2", 9600) ==0) {     //< port no. and baud rate.
      printf(" Congratulations!! You opened the serial port successfully\n");
    }
    else { printf("Exiting due to not opening port\n");
      gpsConn.close();
    }

    //************************************************
    //*********** Reading data from the GPS **********
    //************************************************

    char buf[256];
    long int size ToRead=256;
    int mSecWait =50000;
    unsigned int noOfRead=0;
    int i=0;
    string lati ="";
    string longi="";
    string bear ="";
    string sate ="";

























    /
