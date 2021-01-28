/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Mon Aug 24 2020                                           */
/*    Description:  Distance Sensing                                          */
/*                                                                            */
/*    This program will demonstrate how to use the Distance Sensor            */
/*    to get size, distance, and velocity information of an object            */  
/*    detected in the range of the sensor                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 10           
// Distance2            distance      2               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Print all Distance Sensor values to the screen in an infinite loop
  while (true) {

    // Clear the screen and set the cursor to the top left corner on each loop
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    Brain.Screen.print("Found Object?: ");
    Brain.Screen.print("%s", Distance2.isObjectDetected() ? "TRUE" : "FALSE");
    Brain.Screen.newLine();

    if (Distance2.isObjectDetected()) {
      if (Distance2.objectSize() == sizeType::large) {
        Brain.Screen.print("Object: Large");
      } else if (Distance2.objectSize() == sizeType::medium) {
        Brain.Screen.print("Object: Medium");
      } else if (Distance2.objectSize() == sizeType::small) {
        Brain.Screen.print("Object: Small");
      }

      // Print object distance values in Inches
      Brain.Screen.newLine();
      Brain.Screen.print("Distance in Inches: ");
      Brain.Screen.print("%.2f", Distance2.objectDistance(inches));

      // Print object distance values in MM
      Brain.Screen.newLine();
      Brain.Screen.print("Distance in MM: ");
      Brain.Screen.print("%.2f", Distance2.objectDistance(mm));

      // Print object velocity values (in m/s)
      Brain.Screen.newLine();
      Brain.Screen.print("Object Velocity: ");
      Brain.Screen.print("%.2f", Distance2.objectVelocity());
      Brain.Screen.newLine();
    } else {
      Brain.Screen.print("No Object Detected");
    }

    // A brief delay to allow text to be printed without distortion or tearing
    wait(0.2, seconds);
  }  
}
