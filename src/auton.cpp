#include "vex.h"

void odomTest()
{
    chassis.setInitPos(0, 0, 90);
    while (true)
    {
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(5, 20, "X: %f", chassis.odom.xPos);
        Brain.Screen.printAt(5, 40, "Y: %f", chassis.odom.yPos);
        Brain.Screen.printAt(5, 60, "Heading: %f", chassis.odom.orientation);
        vex::task::sleep(20);
    }
}

void test()
{
    chassis.setInitPos(0, 0, 90);
    chassis.driveToPoint(0, 12, 90); // drive forward 12inch
    chassis.driveToPoint(12, 12, 0); // drive right 12inch
    chassis.turnToHeading(0);        // turn right 90 degree
}