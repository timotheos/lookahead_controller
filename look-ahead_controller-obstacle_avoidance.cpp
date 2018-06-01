//***************************************************************************
//***************************************************************************
// Created by Yu Yu Lwin
//            PhD Student
//            Graduate School of Science and Technology
//            Tokai University
//
// Mobile robot is steering by Look-ahead Controller in the
// Obstacle-avoiding mode
//***************************************************************************
//***************************************************************************

#include "Aria.h"
#include "time.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <fstream>
using namespace std;

fstream wayLa("yuWayPointX.txt", ios:in);
fstream wayLo("yuWayPointY.txt", ios-in);
//****************************************************************************
// Look-ahead Control in Obstacle-avoiding mode
// ***************************************************************************
class ArLookAheadControl {
  public:
    // The constructor, it must use constructor chaining to intialize its base class
    // ArSimpleUserTask
    ArLookAheadControl(ArRobot *robot, double vel, const double gain, double lookahead_distance);
    ~ArLookAheadControl(void) {}  /* Empty destructor */
    void doTask(void);            /* The task we want to do */

  protected:
    ArRobot *myRobot;
    ArActionInput yuAction;
    ArTime myStartTime;
    ArFunctorC<ArLookAheadControl> myTaskCB;

    double y hat x, y_hat_y;
    double y_hat x dot, y_hat_y_dot;
    double Kp, KpOfdudk;
    double u_eql, u_eq2;                    /* new input */
    double phi[3][3];
    double det;
    double inv_phi[3][3];
    double eta_input1, eta_input2;          /* input for mobile robot */
    double robox, roboy;
    double init_ref_x, init_ref_y
    double robophi;
    double x_r, y_r;
    double vel, Rvel;
    double y_eql, y_eq2;

    //**For getting a series of waypoints that represent the desired path
    int ii, no, i;
    double xx, yy, div;
    double wayPtX[10000], wayPtY[10000];
    double dist, distX, distY;            /*distance bet two way points*/
    double myProcessTime[100];
    double L, Lx, Ly, alpha;              /*distance bet COM and current waypoint*/
    double l,l_0;                         /*dist & angle bet lookahead pt and current WP*/
    double delta, init_delta;             /*dist on direction to current WP from Look-ahead point*/
    double obsDist, beta;                 /*distance & angle bet: COM & Obstacle*/
    double lambda, gamma;                 /*dist & angle bet: Look-ahead point & Obstacle*/
    double KK;
};

//*** The constructor, note how it uses chaining to initialize the myTaskCB ***//
ArLookAheadControl::ArLookAheadControl(ArRobot *robot, double vel, const double gain, double
  lookahead distance) : myTaskCB(this, &ArLookAheadControl::doTask) {
    myRobot = robot;
    myStartTime.setToNow();
    myRobot->addUserTask("ArLookAheadControl", 50, &myTaskCB);
    Kp = gain;

    // Initial reference coordinates
    init_ref_x = lookahead_distance;  /* mm */
    init_ref y = 0.0;

    //For getting way points
    ii    = 0;     no    = 0;     div  = vel;
    distX = 0.0;   distY = 0.0;   dist = 0.0;
    Lx    = 0.0;   Ly    = 0.0;   L    = 0.0;
    l     = 0.0;   l_0   = 1500;
    init delta = 400; /* mm */
    KK = 1.0;         /* mm */

    for(i=0; i<100; i++) {myProcessTime[i] = 0.0;}

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
    distX = wayPtx[0] -0.0;
    distY = wayPtY[0] -0.0;
    dist = sqrt((distX * distX) + (distY * distY));
    myProcessTime[0]=dist/div;
}

//*** The main task to perform Look-ahead Control ***//
void ArLookAheadControl::doTask(void) {
    (*myRobot).addAction(&yuAction, 50);

    time_t msec;
AA: msec = myStartTime.mSecSince();
    double myTime = double(msec/1000) + double(msec%1000)/1000;

    if (myTime < myProcessTime[no]+3) {
      robox = myRobot->getX();                      /* mm */
      roboy = myRobot->getY();
      robophi = ArMath::degToRad(myRobot->getTh()); /* rad */

      // Distance between look-ahead point and COM of robot
      x_r = init_ref_x;     y_r = 0.0;

      // Distance & angle between COM & current waypoint
      Lx = wayPtX[no] - robox;
      Ly = wayPtY[no] - roboy;
      L = sqrt( (Lx*Lx) + (Ly*Ly) );
      alpha= atan2(Ly,Lx) - robophi;

      // Distance between look-ahead point & current waypoint
      1 = sqrt( (x_r*x_r) + (L*L) - (2 * x_r*L* cos(alpha)) );

      //*** when robot gets closer to current waypoint, it's chasing it in slow speed
      if(l<= 1000) {
        y_hat_x_dot = 1500 * cos(alpha);
        y_hat_y_dot = 1500 * sin(alpha);
        delta = 1500;
      }
      else {
        y_hat_x_dot = 2000 * cos(alpha);
        y hat y dot = 2000 * sin(alpha);
        delta = 2000;
      }

      y_hat_x = 400; y_hat_y=0;              /* lookahead distance */

      //Coordinates of reference point
      y_hat_x = y_hat_x + y_hat_x_dot * 0.1; /* mm */
      y_hat_y = y_hat_y + y_hat_y_dot * 0.1;

      //*** Getting obstacle's information using SICK
      obsDist = myRobot->checkRangeDevicesCurrentPolar(-70, 70, &beta);

      // Distance and angel between look-ahead point and obstacle
      lambda = sqrt((obsDist * obsDist) + (x_r*x_r) - 2 * obsDist * x_r * cos(ArMath::degToRad(beta)));
      gamma  = asin( (obsDist/lambda)*sin(ArMath::degToRad(beta))); /* rad */

      //*** Obstacle avoidance task
      if(obsDist < 1000) {
        y_eq1 = x_r + delta * cos(alpha) - (KK/lambda) * cos(gamma);
        y_eg2 = delta * sin(alpha) - (KK/lambda) * sin(gamma):
        myProcess Time[no] = myProcessTime[no] + 0.4;
        printf("*\t");
      }


      else {
        y_cql = x_r + delta * cos(alpha);
        y_eq2 = delta * sin(alpha);
      }
      //******

      // New input to the mobile robot
      u_eq1 = y_hat_x_dot + Kp * (y_hat_x - y_eq1);
      u_eq2 = y_hat_y_dot + Kp * (y_hat_y - y_eq2);

      //Inverse of the decoupling matrix Phi for Obstacle-avoiding mode
      inv_phi[1][1] = 1.00;
      inv_phi[1][2] = 0;
      inv_phi[2][1] = 0;
      inv_phi[2][2] = (1.00/x_r);

      //*** controlled input for mobile robot
      eta_inputl = inv_phi[1][1]*u_eq1 + inv_phi[1][2]*u_eq2; /*trans. vel of robot*/
      eta_input2 = inv_phi[2][1]*u_eql + inv_phi[2][2]*u_eq2; /*rotat. vel of robot*/

      vel  = eta_inputl;
      Rvel = ArMath::radToDeg(eta_input2);
      myRobot->lock();
      yuAction.setVel(vel);
      yuAction.setRotVel(Rvel);
      myRobot->unlock());
    }
    else {
      no++;
      if(no<ii) {
        //*** Estimating time and distance between two consecutive waypoints
        distX = wayPEX[no] - wayPtX[no-1];
        distY = wayPtY[mo] - wayPtY[no-1];
        dist = sqrt((distX*distX)+(distY*distY));
        myProcessTime[no] = myProcessTime[no-1] + (dist/div);
        goto AA;
      }
      else {
        printf("OK:Atained GOAL");
        myRobot->lock();
        yuAction.setVel(0);
        yuAction.setRotVel(0);
        myRobot->unlock();
      }
    }
}
//********************** End of ArLookAhead Controller *********************
//**************************************************************************

//**************************************************************************
//*************************** Function Main ********************************
//**************************************************************************

int main(int argc, char **argv) {
  //*** Printing out the program start time.
  time truntime;
  FILE *tim;

  tim = fopen("yuRunTime.txt", "w+");

  if(time(&runtime)= -1) {
    printf("Calendar time not available. \n");
    exit(1);
  }
  else {fprintf(tim, "The start time is \t\t %s \n", ctime(&runtime));}

  //*** Robot and Sick connection.
  ArRobot robot;                    //< the robot
  ArSick sick;                      //< the laser
  ArSerialConnection robotConn;     //< serial connection for Robot
  ArSerialConnection sickConn;      //< serial connection for SICK
  ArTcpConnection tcpConn;          //< Simulation tcp connection
  bool useSim;
  Aria::init();

  //** All the information for our printing out Robot and Sick data.
  double x,y;
  std::list<ArPoseWithTime *> *readings;
  std::list<ArPoseWithTime *>::iterator it;

  FILE  *sickX,*sickY,*robotX,*robot Y, *robotTh;
  sickX   = fopen("yuSickX.txt","w+");
  sickY   = fopen("yuSickY.txt","w+");
  robotX  = fopen("yuRobotX.txt","w+");
  robotY  = fopen("yuRobotY.txt","w+");
  robotTh = fopen("yuRobotThi.txt","w+");

  robot.addRangeDevice(&sick);  //** Add the laser to the robot

  //** See if we can get to the simulator (true is success)
  tcpConn.setPort();
  if(tcpConn.openSimple()) {
    printf("Connecting to simulator throug TCP \n");
    robot.setDeviceConnection(&tcpConn);
    useSim = true;
  }
  else {
    useSim = false;

    //** We could not get to the simulation, so set the port for the
    // serial connection and //then set the serial connection as robot device.
    robotConn.setPort();    //< Default port for robot is COM1
    printf(" Could not connect to the simulator. Connecting to robot throug serial. \n");
    robot.setDeviceConnection(&robotConn);
  }

  //** Try to connect, if we fail exit
  if(!robot.blockingConnect()) {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }

  //** Start the robot running. True so that if we lose connection, the run stops.
  robot.runAsync(true);

  ArActionJoydrive joydrive;
  Joydrive.setStopIfNoButtonPressed(false);
  joydrive.setThrottleParams(100, 400);     /* min and max speed */
  //** Configure and setup SICK sensor
  configureShort(useSim. ArSick::BAUD38400, ArSick::DEGREES 180, ArSick::INCREMENT_ONE);
  sickConn.setPort(ArUtil::COM3);
  sick.setDeviceConnection(&sickConn);
  sick.runAsync();

  //** Check SICK connection
  if(!sick.blockingConnect()) {
    printf("Could not connect to SICK laser... exiting\n");
    Aria::shutdown();
    return 1;
  }
  if(sick.isConnected)) {printf("SICK Connected\n");}
  else                  {printf("SICK: not connected\n");}

  ArUtil::sleep(500);

  //** Turn on the motors, turn off amigobot sounds and turn on joydrive.
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::SOUNDTOG, O);
  robot.comInt(ArCommands::JOYDRIVE, 1);
  robot.addAction(&joydrive, 100);

  //** Call ArLookAhead Control so that robot works with LA controller
  ArLookAheadControl controller( &robot, 200, 1.0, 400);

  while(true) {
    //*** Getting and printing out robot coordinates.
    robot.lock();
    fprintf(robotX,  "%.3f \n",robot.getX());
    fprintf(robotY,  "%.3f \n",robot.getY());
    fprintf(robotTh, "%.3f \n",robot.getTh());
    robot.unlock();

    //*** Getting and printing out Sick readings.
    sick.lockDevice();
    readings = sick.getCurrentBuffer();

    for (it = readings ->begin(); it != readings->end(); it++) {
      x = (*it)->getX()-sick.getSensorPositionX();
      y = (*it)->getY()-sick.getSensorPositionY();
      fprintf(sickX,"%.0f \n",x);
      fprintf(sickY,"%.0f \n",y);
    }

    sick.unlockDevice();
    ArUtil::sleep(100);

    //** Printing out the runtime
    if( time(&runtime) ==-1) {
      printf("Calendar time not available. \n"); exit(1);
    }
    else {fprintf(tim, "* \t The run time is \t %s \n", ctime(&runtime));}
  }

  //** Printing out the end time
  if(time(&runtime) == -1) {
    printf("Calendar time not available. \n");
    exit(1);
  }
  else {fprintf(tim, "The end time is \t\t %s \n", ctime(&runtime));}
  fcloseall();
  robot.waitForRunExit();
  Aria::shutdown();
  exit(1);
  return 0;
}
//*************************** End of Program **********************************
//*****************************************************************************
