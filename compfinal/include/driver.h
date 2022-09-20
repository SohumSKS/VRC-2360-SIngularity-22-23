#ifndef DRIVER_H
#define DRIVER_H
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       auton.h                                                   */
/*    Author:       Sohum Suthar                                              */
/*    Created:      Oct 10 2021                                               */
/*    Description:  Competition Program 2360S                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <vex_competition.h>

using namespace vex;


float maxSpeedPct = 0.8;
bool toggle = false;
void nitroboost() { // Switchable mode
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(0, 0);
  if (maxSpeedPct == 1) {
    maxSpeedPct = 0.8;
    Controller1.rumble("..");
    Controller1.Screen.print("NORMAL SPEED");
  } else {
    maxSpeedPct = 1;
    Controller1.rumble("...");
    Controller1.Screen.print("MAX SPEED");
  }
}

void snailmode() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(0, 0);
  if (maxSpeedPct == 0.5) {
    maxSpeedPct = 0.8;
    Controller1.rumble("..");
    Controller1.Screen.print("NORMAL");
  } else {
    maxSpeedPct = 0.5;
    Controller1.rumble(".");
    Controller1.Screen.print("SLOW");
  }
}
 /* void toggleonoff(){ //toggle to keep the claw engaged while lifting
  if(toggle){
    toggle = false;
  }
  else{
    toggle = true;
  } 
  if(toggle){
    Claw.spin(directionType::fwd, maxSpeedPct, velocityUnits::pct);
  }
  else{
    Claw.stop(hold);
  }
}
*/
void extraSnail() { //slowest switchable mode
  if (Claw.position(degrees) > 60) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.print("EXTRA snail mode");
    maxSpeedPct = 0.25;
  }
}
#endif