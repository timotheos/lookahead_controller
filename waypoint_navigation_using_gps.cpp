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
   return myStrMapsmessage Number).c_str();}

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
  commTimeouts.ReadTotalTimeoutMultiplier = 0;   /* No delay in reading /

























    /
