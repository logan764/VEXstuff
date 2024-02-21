#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor mR1 = motor(PORT3, ratio18_1, false);

motor mArmR = motor(PORT4, ratio18_1, true);

motor mR3 = motor(PORT6, ratio18_1, true);

motor mL3 = motor(PORT8, ratio18_1, false);

motor mL1 = motor(PORT9, ratio18_1, true);

motor mR2 = motor(PORT14, ratio18_1, false);

motor mL2 = motor(PORT18, ratio18_1, true);

motor mArmL = motor(PORT19, ratio18_1, false);

controller Controller1 = controller(primary);
potV2 knob = potV2(Brain.ThreeWirePort.B);
inertial imu = inertial(PORT13);




// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
motor_group arm= motor_group(mArmL, mArmR);
motor_group mLeft = motor_group(mL1, mL2, mL3);
motor_group mRight = motor_group(mR1, mR2, mR3);
smartdrive Drivetrain = smartdrive(mLeft, mRight, imu, 319.19, 255, 235, mm, .5*0.5714285714285714);
// Include the V5 Library
#include "vex.h"
double rerr=0, rlasterr=0, rspeed=0, rsum=0, rkp=.23, rkd=.020, rki=.007;
double terr=0, tlasterr=0, tspeed=0, tsum=0, tkp=.07, tkd=.045, tki=.003;
double tsummax=500;
double maxVoltage=12;
// Allows for easier use of the VEX Library
using namespace vex;

void drivestraight(double target){
  Brain.Screen.newLine();
  target=target*50.13;
  mLeft.setPosition(0,degrees); mRight.setPosition(0,degrees);
  double head=imu.heading(degrees);
  terr=target-(.5*mLeft.position(degrees)+.5*mRight.position(degrees));
  tlasterr=terr;
  int i=0;
  tsum=0;
  rsum=0;

  while(fabs(terr)+fabs(tspeed)>3){
    wait(20,msec);
    i++;
    terr=target-(.5*mLeft.position(degrees)+.5*mRight.position(degrees));
    tspeed=Drivetrain.velocity(rpm); 
    //600rpm * tdk of 0.03 = 30 V.   If the error is 5"*50=250degrees * .1=25V = -5V net
    // We also have sum of like 100ms so 3*err of something, but maxed out?
    //so like 3*1000=4000 *.0003=3V forward.
    //So lets set a max while in motion
    if (fabs(Drivetrain.velocity(percent))<40) {tsum+=terr;}
    else if (tsum>tsummax){tsum=tsummax;}
    else if (tsum<-tsummax){tsum=-tsummax;}


    //tlasterr=terr;


    rerr=head-imu.heading(degrees);
    if (rerr<-180) {rerr+=360;}
    if (rerr>180) {rerr-=360;}
    rspeed=imu.gyroRate(zaxis,dps);
    rlasterr=rerr;
    rsum+=rerr;
    if (fabs(rspeed)>25) {rsum=0;}

    double turnout=rkp*rerr-rkd*rspeed+rsum*rki;
    double driveout=tkp*terr-tkd*tspeed+tsum*tki;

    if (driveout > maxVoltage) {
        driveout=maxVoltage;}
    else if (driveout < -maxVoltage) {
        driveout=-maxVoltage;}

    mLeft.spin(forward,driveout+turnout, volt);
    mRight.spin(forward, driveout-turnout,volt);
    if (i>90) {break; Brain.Screen.print("timeout");}
  }
  Brain.Screen.print("Exit drive with %f", fabs(rerr)+fabs(rspeed));
  mRight.stop();
  mLeft.stop();
}

void turnto(double target){
  int i=0;
  rsum=0;
  rerr=target-imu.heading(degrees);
  Brain.Screen.newLine();
  while(fabs(rerr)+fabs(rspeed)>2){
    wait(20,msec);
    i++;
    
    rerr=target-imu.heading(degrees);
    if (rerr<-180) {rerr+=360;}
    if (rerr>180) {rerr-=360;}
    //rspeed=rerr-rlasterr;
    rspeed=imu.gyroRate(zaxis,dps);
    //rlasterr=rerr;
    rsum+=rerr;
    if (fabs(rspeed)>25) {rsum=0;}
    

    double turnout=rkp*rerr-rkd*rspeed+rki*rsum;
    mLeft.spin(forward,turnout, volt);
    mRight.spin(forward, 0-turnout,volt);
    if (i>200) {break; Brain.Screen.print("timeout");}
  }
  Brain.Screen.print("Exit turn with %f", fabs(rerr)+fabs(rspeed) );
  mRight.stop();
  mLeft.stop();
}


void armto(double target) {
  if (arm.isSpinning()==false) {
    arm.setPosition(fmod(arm.position(degrees),360),degrees);
    }
  double diff = target - arm.position(degrees); // Difference to target

  // Adjust for the shortest path
    if (diff > 180) {
        target -= 360;
    } else if (diff < -180) {
        target += 360;
    }

       // Spin to the calculated target position
    arm.spinToPosition(target, degrees, false); // 'false' for non-blocking, change as needed
}

void auton1() {
  imu.setHeading(0,degrees);
  Drivetrain.setStopping(brake);
  maxVoltage=12;
  turnto(90);
  //wait(500,msec);
  turnto(-30);
  //wait(500,msec);
  turnto(0);
  //wait(500,msec);

  drivestraight(10);
  //wait(500,msec);
  drivestraight(-10);
  //wait(500,msec);
  maxVoltage=4;
  drivestraight(10);
  //wait(500,msec);
  drivestraight(-4);
  maxVoltage=12;
  //wait(500,msec);
  drivestraight(-6);
  //wait(500,msec);

  Drivetrain.setStopping(coast);
  drivestraight(10);
  maxVoltage=3;
  drivestraight(5);
  Drivetrain.setStopping(brake);
  turnto(-80);
  //wait(250,msec);
  turnto(80);
  turnto(180);

  drivestraight(15);
  turnto(0);
  arm.spin(forward);
  wait(1000,msec);
  arm.stop();

}
void auton6() {
  imu.setHeading(0,degrees);
  Drivetrain.setDriveVelocity(90,percent);
  Drivetrain.setTurnVelocity(80,percent);
  arm.setVelocity(100,percent);
  arm.spin(reverse, 7, volt);
  Drivetrain.driveFor(forward, 48, inches,true);

  /*while (Drivetrain.isMoving()){
    if (abs(arm.velocity(percent))<60) {
      armto(300);
      break;
    }
    wait(30,msec);
  }*/
  Drivetrain.setStopping(hold);
  Drivetrain.stop();
  wait(500,msec);
  Drivetrain.turnToHeading(0,degrees,true);
  wait(300,msec);
  Drivetrain.setDriveVelocity(35,percent);
  Drivetrain.driveFor(reverse,3,inches,true); wait(200,msec);

  Drivetrain.turnToHeading(90,degrees,true); wait(200,msec);
  Drivetrain.setDriveVelocity(80,percent);
  Drivetrain.driveFor(forward,24,inches, true);
  arm.spin(reverse,12,volt);   wait(300,msec);

  Drivetrain.turnToHeading(50,degrees); wait(200,msec);
  arm.spin(reverse,6,volt); 

  Drivetrain.setDriveVelocity(30,percent);
  Drivetrain.driveFor(forward,16,inches); //diagonal forward
  arm.spin(reverse,2,volt); //hold?
  wait(200,msec);

  Drivetrain.setDriveVelocity(65,percent);
  Drivetrain.driveFor(reverse,5,inches,true);//backup
  wait(200,msec);
  Drivetrain.turnToHeading(90,degrees,true);//turn
  wait(200,msec);
  arm.spinFor(forward,220,degrees,true); //ready set
  arm.spin(reverse,12,volt);//pop it out
  wait(500,msec);
  Drivetrain.turnToHeading(225,degrees);
  arm.stop(); wait(200,msec);
  armto(60);

  Drivetrain.setStopping(coast);
  Drivetrain.setDriveVelocity(100,percent);
  Drivetrain.driveFor(forward, 50,inches);
  Drivetrain.stop();
  wait(200,msec);
  mLeft.spin(forward,3,volt);
  mRight.spin(forward,3,volt);
  wait(500,msec);
  while(true){
    wait(20,msec);
    if ((fabs(mLeft.velocity(percent))<15)||(fabs(mLeft.velocity(percent))<15)){break;}
    }
  Drivetrain.stop();

  Drivetrain.turnToHeading(-45,degrees);
  wait(200,msec);
  Drivetrain.driveFor(20,inches, true);
  mLeft.spin(forward,5,volt);
  mRight.spin(forward,3,volt);
  wait(350,msec);
  Drivetrain.turnToHeading(0,degrees);
  Drivetrain.drive(forward);
  wait(1000,msec);
  Drivetrain.drive(reverse);
  wait(500,msec);
  Drivetrain.turnToHeading(180,degrees);
  arm.spin(reverse,8,volt);
  Drivetrain.driveFor(forward,24, inches);
  arm.spin(reverse,2,volt);
  mRight.spin(forward,12,volt);
  wait(500,msec);
  Drivetrain.setTimeout(1,seconds);
  Drivetrain.turnToHeading(90,degrees);
  Drivetrain.setTimeout(2.5,seconds);
  wait(500,msec);

  arm.spin(forward,12,volt);
  Drivetrain.driveFor(forward, 20,inches); //gun it
  wait(1000,msec);
  arm.spin(forward,2,volt);
  Drivetrain.setDriveVelocity(20,percent);
  Drivetrain.driveFor(forward,4,inches);
  wait(1000,msec);
  }

void auton2(){
  imu.setHeading(0,degrees);
  Drivetrain.setDriveVelocity(70,percent);
  Drivetrain.setTurnVelocity(10,percent);
  
  arm.setVelocity(100,percent);
  arm.spin(reverse, 7, volt);
  Drivetrain.driveFor(forward, 24, inches,true);
  Drivetrain.setStopping(hold);
  Drivetrain.stop();
  wait(500,msec);
  Drivetrain.turnToHeading(0,degrees,true);
  wait(300,msec);
  Drivetrain.setDriveVelocity(35,percent);
  Drivetrain.driveFor(reverse,3,inches,true); wait(200,msec);
  Drivetrain.turnToHeading(90,degrees);
  arm.spin(forward);
  wait(250,msec);
  Drivetrain.driveFor(reverse,18,inches);
  arm.spin(reverse,7,volt);
  Drivetrain.turnToHeading(-45,degrees);
  Drivetrain.setDriveVelocity(40,percent);
  Drivetrain.driveFor(forward,10,inches);
  arm.spin(reverse,4,volt);
  Drivetrain.turnToHeading(0, degrees);
  Drivetrain.driveFor(forward, 8,inches);
  arm.spin(reverse,2,volt);
  //Drivetrain.driveFor(reverse,)

  

}
void auton3(){}
void auton4(){}

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  imu.calibrate();
  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  // place automonous code here
  if(knob.angle(percent)<25){auton1();}
  else if(knob.angle(percent)<50){auton2();}
  else if(knob.angle(percent)<75){auton3();}
  else {auton4();}
}

void userControl(void) {
  Brain.Screen.clearScreen();
  wait(20, msec);
  arm.setMaxTorque(100,percent);
  arm.setVelocity(100,percent);
  while (true){
    wait(20,msec);

    if (Controller1.ButtonL1.pressing()&&Controller1.ButtonL2.pressing()) {armto(120);}
    else if (Controller1.ButtonL1.pressing()) {arm.spin(reverse,-12,volt);}
    else if (Controller1.ButtonL2.pressing()) {armto(45);}
    else if (Controller1.ButtonR1.pressing()&&Controller1.ButtonR2.pressing()) {arm.setStopping(hold); arm.stop();}
    else if (Controller1.ButtonR1.pressing()) {arm.spin(reverse,10,volt);}
    else if (Controller1.ButtonR2.pressing()) {arm.spin(reverse,4,volt);}
    else {arm.setStopping(coast);    arm.stop();}
    mLeft.spin(forward,Controller1.Axis3.position()*.12+Controller1.Axis1.position()*.75*.12, volt);
    mRight.spin(forward,Controller1.Axis3.position()*.12-Controller1.Axis1.position()*.75*.12, volt);


   //Auton  Test Code: Delete
    if (Controller1.ButtonDown.pressing()){auton1();}
    if (Controller1.ButtonLeft.pressing()){auton2();}
    if (Controller1.ButtonB.pressing()){arm.setPosition(0,degrees);}
  }
}

int main() {
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}