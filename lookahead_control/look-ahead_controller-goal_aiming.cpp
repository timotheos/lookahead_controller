//***************************************************************************
//***************************************************************************
// Created by Yu Yu Lwin
//            PhD Student
//            Graduate School of Science and Technology
//            Tokai University
//
// Mobile robot is steering by Look-ahead Controller in the Goal-aiming
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

using namespace std;
#define RAD (180/3.1415926535897)
fstream wayLa("yuWayPointX.txt", ios::in);       /* Geting X_Waypoints of the desired route */
fstream WayLo("yuWayPointY.txt", ios::in);      /* Geting Y_Waypoints of the desired route */
FILE *rx, *ry, *rt;


//***************************************************************************
//*************  Look-ahead Controller in Goal-aiming mode  *****************
//***************************************************************************


//*** Class of ArLookAheadControl ***//
class ArLookAhead Control {
  public:
    /* the constructor, it must use constructor chaining to intialize its base */
    ArLookAheadControl(ArRobot *robot, double vel, const double gain, double lookahead_distance);

    ~ArLookAheadControl(void) {}      /* empty destructor */
    void doTask(void);                /* the task we want to do */

  protected:
    ArRobot *myRobot;
    ArActionInput yuAction;
    ArTime myStartTime;
    ArFunctorC<ArLookAheadControl>myTaskCB;

    double y_hat_x, y_hat_y;
    double y_hat_x_dot, y_hat_y_dot;
    double Kp, KpOfdudk;
    double u_eq1, u_eq2;              /* new input */
    double phi[3][3];
    double det;
    double inv_phi[3][3];
    double eta_input1, eta_input2;    /* control input for mobile robot */
    double robox, roboy;
    double init_ref_x, init_ref_y;
    double robophi;
    double x_r, y_r;
    double vel, Rvel;
    double y_eq1, y_eq2;
    double omiga;
    double radius;

    FILE *refx, *refy;
    double wayPtX[1000], wayPtY[1000];
    double xx, yy;
    double L, Lx, Ly, alpha, 1;       /* distace between COM and next waypoint */
    int    ii, no, i;
    double dist, distX, distY , div;  /* distace between two way points */
    double myProcess Time[100];

    double wx, wy;
    double rbx, rby, rbt;
};

//*** The constructor, note how it uses chaining to initialize the myTaskCB ***//
/* constructor is a special type of subroutine called to create an object. It
prepares the new object for use, often accepting arguments that the constructor
uses to set required member variables.  */
ArLookAheadControl::ArLookAheadControl(ArRobot *robot, double vel, const double gain, double lookahead_distance) :
  myTaskCB(this, &ArLookAheadControl::doTask) {
    myRobot = robot;

    myStartTime.setToNow();
    // Just add this user task to the Robot
    myRobot->addUserTask("ArLookAheadControl", 50, &myTaskCB);

    Kp = gain;
    // Initial reference coordinate
    init_ref_x = lookahead_distance;    /* mm */
    init_ref_y = 0.0;
    refx = fopen("yuReferencePoint X.txt","w+");
    refy = fopen("yuReferencePoint Y.txt","w+");
    rx   = fopen("yuRobotX.txt", "w+");
    rt   = fopen("yuRobot Thi.txt", "w+");

    Lx      = 0.0;     Ly      =0.0;      L   = 0.0;
    alpha   = 0.0;     l       =0.0;
    y_hat_x = 400;     y_hat_y =0
    ii      = 0;       no      = 0.0;     div = vel;

    // Initializing the consecutive travelling time with zero
    for(i=0; i<100; i++)    myProcess Time[i] = 0.0;

    if(!wayLa || !wayLo) {
      printf(" Exit due to not having way point files \n");
      exit(2);
    }
    else {
      while(! wayLa.eof() || !wayLo.eof()) {
        wayLa>>xx;    wayPtX[ii] = xx;
        wayLo>>yy;    wayPtY[ii] = yy;
        ii++;
      }
    }

    printf(" Number of way points : %d \n\n", ii);
    dist = sqrt((wayPtX[0] * wayPtX[0]) + (wayPtY[0] * wayPtY[0]));
    myProcessTime[0] = dist/div;

    wx = 0.0;   wy = 0.0;
    rbx = 0.0; rby = 0.0; rbt =0.0;
}

//*** The main task to perform Look-ahead Control ***//
void ArLookAheadControl::doTask(void) {
  (*myRobot).addAction(&yuAction, 50);
  time_t msec;
  AA:  msec = myStartTime.mSecSince();
  double myTime = double(msec/1000) + double(msec%1000)/1000;
  printf("%.31f\n",myTime);

  // Checking with both the travelling time and the number of waypoints
  if ((myTime < myProcessTime[no]) && no <ii) {
    robox   = myRobot->getX();  /* mm */
    roboy   = myRobot->getY();  /* rad */
    robophi = ArMath::degToRad(myRobot->getTh());
    rbx = robox;
    rby = roboy;
    rbt = robophi;
    fprintf(rx, "%f \n", robox);
    fprintf(ry, " %f \n", roboy);
    fprintf(rt, " %f \n", robophi);

    //*** Distance between look-ahead point and COM of robot
    x_r= init_ref_x;    y_r= init_ref_y;

    //*** Distance & angle between COM & current waypoint
    Lx = wayPtX[no] - robox;
    Ly = wayPtY[no] - roboy;
    L  = sqrt( (Lx * Lx) + (Ly * Ly));
    alpha = atan2(Ly,Lx);

    //*** Distance between Look-ahead point & current waypoint
    l  = sqrt( (x_r*x_r) + (L*L) - (2*x_r*L*cos(alpha)));

    //*** Just changing the velocities of x and y directions @ reference point
    y_hat_x_dot = 300 * cos(alpha);
    y_hat_y_dot = 300 * sin(alpha);

    //*** Coordinates of the reference point
    // when robot closes to current waypoint (< 500 mm from WP), velocity
    // is going to slow down. Just for knowing the action of chasing WP
    if (L < 500) {
      y_hat_x = robox + (0.75) * x_r;
      y_hat_y = roboy + (0.75) * y r;
      printf(" ** "); }
    else {
      y_hat_x = robox + x_r;
      y_hat_y = roboy + y_r;
    }

    //*** Controlled output equation for the Goal-aiming mode
    y_eq1 = robox + x_r* cos(robophi) - y_r* sin(robophi);
    y_eq2 = roboy + x_r* sin(robophi) + y_r* cos(robophi);

    //*** New input to the mobile robot
    u_eq1 = y hat x dot + Kp * (y_hat_x - y_eq1);
    u_eq2 = y_hat_y_dot + Kp * (y_hat_y - y_eq2);

    //*** Decoupling matrix for the Goal-aiming mode
    phi[1][1] = cos(robophi);
    phi[1][2] = -x_r * sin(robophi) - y_r * cos(robophi);
    phi[2][1] = sin(robophi);
    phi[2][2] =  x_r * cos(robophi) - y_r * sin(robophi);

    //*** Inverse of decoupling matrix
    det = phi[1][1] * phi[2][2] - phi[2][1] * phi[1][2];    /* determinant of phi */
    inv_phi[1][1] =  phi[2][2]/det;
    inv_phi[1][2] = -phi[1][2]/det;
    inv_phi[2][1] = -phi[2][1]/det;
    inv_phi[2][2] =  phi[1][1]/det;

    //*** Control input for mobile robot
    eta_input1 = inv_phi[1][1] * u_eq1 + inv_phi[1][2] * u_eq2; /*trans. vel of robot*/
    eta_input2 = inv_phi[2][1] * u_eq1 + inv_phi[2][2] * u_eq2; /*rot. vel of robot*/

    //*** Giving the controlled translational and rotational velocities to the mobile
    //    robot via ARIA
    vel = eta_input1;
    Rvel = ArMath::radToDeg(eta_input2);
    myRobot->lock();
    yuAction.setVel(vel);¥
    yuAction.setRotVel(Rvel);
    myRobot->unlock():

    fprintf(rx, " %f \n", rbx);
    fprintf(ry, " %f \n", rby);
    fprintf(rt, " %f \n", rbt);

    //*** Checking next waypoint to attain *** //
    // when robot enters the waypoint's circle (here 100 mm radius),
    // it is gonna move to attain next waypoint
    wx = wayPtX[no];    wy = wayPtY[no];
    if((wx-100 < robox) && (robox <wx+100) && (wy-100 < roboy) && (roboy <wy+100)) {
      printf("\t***");
      no++;
      distX = wayPtX[no] - wayPtX[no-1];
      distY = wayPtY[no] - wayPtY[no-1];
      dist = sqrt((distX * distX) + (distY * distY)); /*dist bet. 2 consecutive WP*/

      // estimating the time which may take for attaining the next waypoint
      // from current one
      myProcessTime[no] = myProcessTime[no-1] + (dist/div);
      goto AA;
    }
  }
  else {
    printf("OK : Attained the desired path");
    myRobot->lock();
    yuAction.setVel(0);
    yuAction.setRotVel(0);
    myRobot->unlock();
  }
}
//********************** End of ArlookAhead Controller **********************
//***************************************************************************

//***************************************************************************
//*************************** Function Main *********************************
//***************************************************************************

int main(int argc. char **argy) {
  //*** Printing out the program start time.
  time_t runtime;
  FILE *tim;
  tim = fopen("yuRun Time.txt","w+");

  if( time(&runtime) == -1) {
    printf("Calendar time not available. \n");
    exit(1);
  }
  else fprintf(tim. "The start time is \t\t %s \n", ctime(&runtime));
  //** Configurations for Robot and Sick connections.
  ArRobot robot;                /* the robot */
  ArSick sick;                  /* the laser */
  ArSerialConnection robotConn; /* the robot serial connection */
  ArSerialConnection sickConn;  /* SICK serial connection */
  ArTcpConnection tcpConn;      /* Simulation TCP connection */
  bool useSim;
  Aria::init();

  //** All the information for our printing out Robot and Sick data.
  double x,y;
  std::list<ArPoseWithTime *> *readings;
  std::list<ArPoseWithTime *>::iterator it;

  FILE *sickX,*sickY;
  sickX = fopen("yuSickX.txt", "w+");
  sick Y = fopen("yuSickY.txt","w+");

  //** Add the laser to the robo
  robot.addRangeDevice(&sick);

  //** See if we can get to the simulator (true is success)
  tcpConn.setPort(); if(tcpConn.openSimple()) {
    printf("Connecting to simulator throug TCP \n");
    robot.setDeviceConnection(&tcpConn);
    useSim = true;
  }
  else {
    useSim = false;
    // We could not get to the simulation, so set the port for
    // the serial connection and then set the serial connection
    // as robot device.
    robotConn.setPort();
    // Default port for robot is COM1
    printf(" Could not connect to the simulator. Connecting to robot throug serial. \n");
    robot.setDeviceConnection(&robotConn);
  }

  //** Try to connect. If we fail, exit the program.
  if(!robot.blockingConnect()) {
      printf("Could not connect to robot... exiting\n");
      Aria::shutdown();
      return 1;
  }

  //** Start the robot running. True so that if we lose connection, the run stops.
  robot.runAsync(true);

  //** Joystick.
  ArActionJoydrive joydrive;
  joydrive.setStoplfNoButtonPressed(false);
  joydrive.setThrottleParams(100, 400);       /* min and max speed */

  //** Configure the SICK before Logger and connection
  sick.configureShort(useSim, ArSick::BAUD38400, ArSick::DEGREES180,
      ArSick::INCREMENT ONE);

  //** For creating map file using robot and sick reading.
  ArSickLogger logger(&robot, &sick, 300, 25, filename.c_str(), true);

  //** Now set up the laser
  sickConn.setPort(ArUtil::COM3);
  sick.setDeviceConnection(&sickConn);
  sick.runAsync();

  //** Check SICK connection
  if(!sick.blockingConnect()) {
    printf("Could not connect to SICK laser... exiting\n");
    Aria::shutdown();
    return 1;
  }
  if(sick.isConnected()) {printf("SICK Connected\n");}
  else {printf("SICK: not connected\n");}
  ArUtil::sleep(500);

  //** Turn on the motors, turn off amigobot sounds and turn on joydrive.
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::SOUNDTOG, 0);
  robot.comInt(ArCommands::JOYDRIVE, 1);

  //** Add the wander actions.
  robot.addAction(&joydrive, 100);

  //** Call ArLookAheadControl function so that rb works with Look-ahead controller
  ArLookAheadControl controller( &robot, 200, 1.0, 400 );

  while(true) {
    sick.lockDevice();
    readings = sick.getCurrentBuffer(); //< Getting current buffer in which sick
                                        // reading is stored.
    for (it = readings ->begin(); it != readings->end(); it++) {
      x = (*it)->getX()-sick.getSensorPositionX():
      y = (*it)->getY()-sick.getSensorPositionY();
      fprintf(sickX,"%.0f \n",x);
      fprintf(sickY,"%.0f \n",y);
    }

    sick.unlockDevice();
    ArUtil::sleep(100);

    //** Printing out the runtime
    if( time(&runtime)=-1) {
      printf("Calendar time not available. \n");
      exit(1);
    }
    else {fprintf(tim,"* \t The run time is \t%s \n", ctime(&runtime));}
  }

  //** Printing out the end time
  if (time(&runtime)=-1) {
    printf("Calendar time not available. \n");
    exit(1);
  }
  else fprintf(tim, "The end time is \t\t %s \n", ctime(&runtime));

  fcloseall();
  robot.waitForRunExit(); //< before exiting the program, close all ARIA utilities
  Aria::shutdown();  return 0;  exit(1);
}
//*************************** End of Program **********************************
//*****************************************************************************
