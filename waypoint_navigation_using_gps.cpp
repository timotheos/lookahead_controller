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
};






















    /
