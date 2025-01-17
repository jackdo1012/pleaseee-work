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

Drive chassis = Drive(9, 11, 12, vex::motor_group(leftBackMotor, leftFrontMotor),
                      vex::motor_group(rightBackMotor, rightFrontMotor));
// Drive drive = Drive(
//     // left motors
//     leftMotors,
//     // right motors
//     rightMotors,
//     // inertial port
//     PORT9,
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
    armSensor.setPosition(0, vex::rotationUnits::rev);
    leftMotors.setStopping(vex::brake);
    rightMotors.setStopping(vex::brake);
    intakeMotor.setVelocity(100, percent);
    conveyorMotor.setVelocity(90, percent);
    leftArmMotor.setVelocity(70, percent);
    rightArmMotor.setVelocity(70, percent);
}

enum MotorStatus
{
    forward,
    reverse,
    stop
};

// off, loading, neutral_hover, neutral_score
double armPoses[] = {0, 43, 258, 290};

MotorStatus intakeStatus = MotorStatus::stop;
int armPos = 0;
double armTarget = 0;
int numberOfPos = 4;
double armPrevErr = 0;
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
    if (powerStatus)
    {
        return;
    }
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
    if (powerStatus)
    {
        return;
    }
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
    std::string printData = "Clamp: " + std::string(clampStatus ? "on" : "off");
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
    ptoMotor.setStopping(coast);
    wait(0.5, sec);
    while ((ptoMotor.torque() < 0.2 && powerStatus) || intakeMotor.torque() < 0.2)
    {
        wait(5, msec);
        if (ptoMotor.torque() >= 0.2 || !powerStatus)
        {
            ptoMotor.stop();
        }
        if (intakeMotor.torque() >= 0.2)
        {
            intakeMotor.stop();
        }
    }
    powerProcessing = false;
    std::string printData2 = "Power: " + std::string(powerStatus ? "on" : "off");
    printOnController(2, 1, printData2);
}

void armControl()
{
    double kp = 0.5;
    double err = armTarget - (armSensor.position(vex::rotationUnits::rev) * 360);
    double output = kp * err;
    output = clamp(-100, output, 100);
    leftArmMotor.spin(vex::forward, output, percent);
    rightArmMotor.spin(vex::forward, output, percent);
}

int armMovementTask()
{
    armSensor.setPosition(0, vex::rotationUnits::rev);
    while (true)
    {
        armControl();
        vex::task::sleep(20);
    }
    return 0;
}

void armOut()
{
    if (armPos < numberOfPos - 1)
    {

        armPos += 1;
        armTarget = armPoses[armPos];
    }
}

void armIn()
{
    if (armPos > 0)
    {
        armPos--;
        armTarget = armPoses[armPos];
    }
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
    chassis.setInitPos(0, 0, 90);
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

void usercontrol()
{
    vex::task armTask(armMovementTask);
    // Controller.ButtonX.pressed(togglePower);
    Controller.ButtonA.pressed(toggleClamp);
    Controller.ButtonR1.pressed(armOut);
    Controller.ButtonR2.pressed(armIn);
    Controller.ButtonL1.pressed(runIntake);
    Controller.ButtonL2.pressed(reverseIntake);
    Controller.Screen.clearScreen();
    // User control code here, inside the loop
    while (1)
    {
        double leftSpeed = Controller.Axis3.position() + Controller.Axis1.position();
        double rightSpeed = Controller.Axis3.position() - Controller.Axis1.position();

        if (fabs(leftSpeed) < 5)
        {
            leftSpeed = 0;
        }
        if (fabs(rightSpeed) < 5)
        {
            rightSpeed = 0;
        }
        leftMotors.spin(vex::forward, leftSpeed, percent);
        rightMotors.spin(vex::forward, rightSpeed, percent);
        if (powerStatus)
        {
            intakeMotor.spin(vex::forward, leftSpeed, percent);
            ptoMotor.spin(vex::forward, rightSpeed, percent);
        }
        wait(20, msec); // Sleep the task for a short amount of time
                        // to prevent wasted resources.
    }
}

int odomTest()
{
    while (true)
    {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %f", chassis.odom.xPos);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %f", chassis.odom.yPos);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Orientation: %f", chassis.odom.orientation);
        vex::task::sleep(20);
    }
    return 0;
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

    // chassis.setInitPos(0, 0, 270);
    // chassis.driveToPoint(0, 19, 270, 3000);
    // toggleClamp();
    // runIntake();
    // chassis.turnToHeading(45, 10000);

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
