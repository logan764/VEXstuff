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
controller Controller1 = controller(primary);
motor mArm = motor(PORT1, ratio18_1, false);

motor mLeft = motor(PORT2, ratio18_1, false);

motor mRight = motor(PORT3, ratio18_1, true);




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

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

double armOut, driveOut, turnOut;

int main() {
  while(true){
    if(Controller1.ButtonL1.pressing()){
      armOut = 5;
    }
    else if(Controller1.ButtonL2.pressing()){
      armOut =-5;
    }
    else{
      armOut = 0;
    }
    mArm.spin(forward,armOut,volt);

    driveOut=Controller1.Axis3.position()*0.12;
    turnOut=Controller1.Axis1.position()*0.12*0.7;
    mLeft.spin(forward,driveOut+turnOut,volt);
    mRight.spin(forward,driveOut-turnOut,volt);
  }
}
