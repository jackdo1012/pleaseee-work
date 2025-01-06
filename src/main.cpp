/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jackdo1012                                                */
/*    Created:      12/16/2024, 7:58:48 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "iostream"
#include "vex.h"

using namespace vex;

competition Competition;

Drive chassis = Drive();
// Drive drive = Drive(
//     // left motors
//     leftMotors,
//     // right motors
//     rightMotors,
//     // inertial port
//     PORT7,
//     // vertical odo port
//     PORT11,
//     // horizontal odo port
//     PORT12,
//     // left back motor port
//     PORT1,
//     // left front motor port
//     PORT2,
//     // right back motor port
//     PORT3,
//     // right back motor port
//     PORT4);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions */
/*                                                                           */
/*  You may want to perform some actions before the competition starts. */
/*  Do them in the following function.  You must return from this function
 */
/*  or the autonomous and usercontrol tasks will not be started.  This */
/*  function is only called once after the V5 has been powered on and */
/*  not every time that the robot is disabled. */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{
    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
    Inertial.calibrate(3);
    while (Inertial.isCalibrating())
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Inertial Calibrating");
        wait(50, msec);
    }
    Brain.Screen.clearScreen();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
    // ..........................................................................
    // Insert autonomous user code here.
    // ..........................................................................
    odomTest();
    // test();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

enum MotorStatus
{
    forward,
    reverse,
    stop
};

MotorStatus intakeStatus = MotorStatus::stop;
bool clampStatus = false;
bool powerStatus = false;
bool powerProcessing = false;

void printOnController(int row, int col, std::string data)
{
    Controller.Screen.clearLine(row);
    Controller.Screen.setCursor(row, col);
    Controller.Screen.print(data.c_str());
}

void runIntake()
{
    if (intakeStatus == MotorStatus::forward)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::forward);
        conveyorMotor.spin(vex::forward);
        intakeStatus = MotorStatus::forward;
    }
}

void reverseIntake()
{
    if (intakeStatus == MotorStatus::reverse)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::reverse);
        conveyorMotor.spin(vex::reverse);
        intakeStatus = MotorStatus::reverse;
    }
}

void toggleClamp()
{
    clampStatus = !clampStatus;
    clampCylinder.set(clampStatus);
    std::string printData = "Clamp: " + std::string(powerStatus ? "on" : "off");
    printOnController(1, 1, printData);
}

void togglePower()
{
    if (powerProcessing)
    {
        return;
    }
    powerProcessing = true;
    powerStatus = !powerStatus;
    powerCylinder.set(powerStatus);
    intakeMotor.setStopping(coast);
    conveyorMotor.setStopping(coast);
    wait(0.5, sec);
    while (conveyorMotor.torque() < 0.2 || intakeMotor.torque() < 0.2)
    {
        wait(5, msec);
        if (conveyorMotor.torque() >= 0.2)
        {
            conveyorMotor.stop();
        }
        if (intakeMotor.torque() >= 0.2)
        {
            intakeMotor.stop();
        }
    }
    std::string printData = "Power: " + std::string(powerStatus ? "on" : "off");
    printOnController(2, 1, printData);
    powerProcessing = false;
}

void usercontrol()
{
    // User control code here, inside the loop
    while (1)
    {
        int leftSpeed = Controller.Axis3.position() + Controller.Axis1.position();
        int rightSpeed = Controller.Axis3.position() - Controller.Axis1.position();

        if (abs(leftSpeed) < 5)
        {
            leftSpeed = 0;
        }
        if (abs(rightSpeed) < 5)
        {
            rightSpeed = 0;
        }
        leftMotors.spin(vex::forward, leftSpeed, percent);
        rightMotors.spin(vex::forward, rightSpeed, percent);
        if (powerStatus)
        {
            intakeMotor.spin(vex::forward, leftSpeed, percent);
            conveyorMotor.spin(vex::forward, rightSpeed, percent);
        }
        else
        {
            intakeMotor.setVelocity(100, percent);
            conveyorMotor.setVelocity(100, percent);
            Controller.ButtonL1.pressed(runIntake);
            Controller.ButtonL2.pressed(reverseIntake);
        }

        Controller.ButtonX.pressed(togglePower);

        wait(20, msec); // Sleep the task for a short amount of time
                        // to prevent wasted resources.
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
