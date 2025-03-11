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
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Inertial Calibrating");
        wait(50, msec);
    }
    Brain.Screen.clearScreen();
    Controller.Screen.clearScreen();
    leftMotors.setStopping(vex::brake);
    rightMotors.setStopping(vex::brake);
    intakeMotor.setVelocity(100, percent);
    conveyorMotor.setVelocity(100, percent);
    leftArmMotor.setVelocity(70, percent);
    rightArmMotor.setVelocity(70, percent);
}

enum MotorStatus
{
    forward,
    reverse,
    stop
};

// off, loading, neutral
// double armPoses[] = {0, 50, 279, 380};
// double armPoses[] = {0, 52, 150, 279};
double armPoses[] = {0, 52, 279};

int numberOfPos = 3;
MotorStatus intakeSystemStatus = MotorStatus::stop;
int armPos = 0;
double armTarget = 0;
double armPrevErr = 0;
bool clampStatus = false;
bool doinkerStatus = false;
bool powerProcessing = false;
bool slowIntake = false;
int pos = 1; // 1: red left, 2: red right, 3: blue left, 4: blue right
bool colorSorting = true;

void printOnController(int row, int col, std::string data)
{
    Controller.Screen.clearLine(row);
    Controller.Screen.setCursor(row, col);
    Controller.Screen.print(data.c_str());
}

void toggleIntakeOnly()
{
    intakeMotor.spin(vex::forward);
}

void runIntake()
{
    if (intakeSystemStatus == MotorStatus::forward)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeSystemStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::forward);
        conveyorMotor.spin(vex::forward);
        intakeSystemStatus = MotorStatus::forward;
    }
}

void reverseIntake()
{
    if (intakeSystemStatus == MotorStatus::reverse)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeSystemStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::reverse);
        conveyorMotor.spin(vex::reverse);
        intakeSystemStatus = MotorStatus::reverse;
    }
}
void toggleClamp()
{
    clampStatus = !clampStatus;
    clampCylinder.set(clampStatus);
}

void toggleDoinker()
{
    doinkerStatus = !doinkerStatus;
    doinkerCylinder.set(doinkerStatus);
}

void armControl()
{
    double kp = .5;
    double kd = .2;
    double err = armTarget - (leftArmMotor.position(vex::rotationUnits::rev) * 360);
    double output = kp * err + kd * (err - armPrevErr);
    armPrevErr = err;
    output = clamp(-50, output, 50);
    leftArmMotor.spin(vex::forward, output, percent);
    rightArmMotor.spin(vex::forward, output, percent);
}

int armMovementTask()
{
    leftArmMotor.setPosition(0, vex::rotationUnits::rev);
    while (true)
    {
        armControl();
        vex::task::sleep(10);
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

void slowIntakeControl()
{
    conveyorMotor.setVelocity(5, percent);
}

void fastIntakeControl()
{
    conveyorMotor.setVelocity(100, percent);
}

void sideSelection()
{
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 240, 240);
    Brain.Screen.setCursor(3, 8);
    Brain.Screen.print("red left");

    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(240, 0, 240, 120);
    Brain.Screen.setCursor(3, 32);
    Brain.Screen.print("red right");

    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(0, 120, 240, 120);
    Brain.Screen.setCursor(10, 8);
    Brain.Screen.print("blue left");

    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(240, 120, 240, 120);
    Brain.Screen.setCursor(10, 32);
    Brain.Screen.print("blue right");

    while (!Brain.Screen.pressing())
    {
        wait(5, msec);
    }

    if (Brain.Screen.xPosition() < 240)
    {
        if (Brain.Screen.yPosition() < 120)
        {
            pos = 1;
            Brain.Screen.setFillColor(red);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
            armTarget = armPoses[1];
        }
        else
        {
            pos = 3;
            Brain.Screen.setFillColor(blue);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
        }
    }
    else
    {
        if (Brain.Screen.yPosition() < 120)
        {
            pos = 2;
            Brain.Screen.setFillColor(red);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
        }
        else
        {
            pos = 4;
            Brain.Screen.setFillColor(blue);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
            armTarget = armPoses[1];
        }
    }
}
void colorSortingFunc()
{
    if (colorSensor.color() < 1000000 && pos - 2 > 0)
    {
        return;
    }
    else if (colorSensor.color() > 1000000 && pos - 2 <= 0)
    {
        return;
    }
    if (colorSorting)
    {
        wait(50, msec);
        conveyorMotor.stop(vex::brake);
        wait(1, seconds);
        conveyorMotor.spin(vex::forward);
        intakeMotor.spin(vex::forward);
        intakeSystemStatus = MotorStatus::forward;
    }
}

void toggleColorSorting()
{
    colorSorting = !colorSorting;
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
    // 1 : red ring, 2 : red stake, 3 : blue stake, 4 : blue ring
    if (pos == 1)
    {

        chassis.setInitPos(0, 0, 270);
        chassis.changeDrivePid(1, 0, 10);
        chassis.changeTurnPid(0.8, 0, 8);
        chassis.driveToPoint(0, 19, 2000);
        clampCylinder.set(true);
        wait(500, msec);
        runIntake();

        chassis.turnToHeading(124, 2000);
        chassis.driveToPoint(-15, 40, 3000, 100);
    }
    else if (pos == 2)
    {
        chassis.setInitPos(0, 0, 270);
        chassis.changeDrivePid(1, 0, 10);
        chassis.changeTurnPid(0.8, 0, 8);
        chassis.driveToPoint(0, 19, 2000);
        clampCylinder.set(true);
        wait(500, msec);
        runIntake();
        chassis.turnToHeading(20, 2000);
        chassis.driveToPoint(22 * cos(toRadian(20)), 19 + 22 * sin(toRadian(20)), 2000);
        wait(1000, msec);
        runIntake();
        chassis.stop();

        chassis.turnToHeading(90, 2000);
        chassis.driveToPoint(22 * cos(toRadian(20)), 19 + 22 * sin(toRadian(20)) - 10, 2000);
        clampCylinder.set(false);
        chassis.driveToPoint(22 * cos(toRadian(20)), 19 + 22 * sin(toRadian(20)), 2000);
        chassis.turnToHeading(270, 2000);
    }
    else if (pos == 3)
    {
        chassis.setInitPos(0, 0, 270);
        chassis.changeDrivePid(1, 0, 10);
        chassis.changeTurnPid(0.8, 0, 8);
        chassis.driveToPoint(0, 19, 2000);
        clampCylinder.set(true);
        wait(500, msec);
        runIntake();
        chassis.turnToHeading(160, 2000);
        chassis.driveToPoint(22 * cos(toRadian(160)), 19 + 22 * sin(toRadian(160)), 2000);
        wait(1000, msec);
        runIntake();
        chassis.stop();

        chassis.turnToHeading(90, 2000);
        chassis.driveToPoint(22 * cos(toRadian(160)), 19 + 22 * sin(toRadian(160)) - 10, 2000);
        clampCylinder.set(false);
        chassis.driveToPoint(22 * cos(toRadian(160)), 19 + 22 * sin(toRadian(160)), 2000);
        chassis.turnToHeading(270, 2000);
    }
    else if (pos == 4)
    {
        chassis.setInitPos(0, 0, 270);
        chassis.changeDrivePid(1, 0, 10);
        chassis.changeTurnPid(0.8, 0, 8);
        chassis.driveToPoint(0, 19, 2000);
        clampCylinder.set(true);
        wait(500, msec);
        runIntake();

        chassis.turnToHeading(56, 2000);
        chassis.driveToPoint(15, 40, 3000, 100);
    }
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
    Controller.ButtonA.pressed(toggleClamp);
    Controller.ButtonDown.pressed(toggleDoinker);
    Controller.ButtonR1.pressed(armOut);
    Controller.ButtonR2.pressed(armIn);
    Controller.ButtonL1.pressed(runIntake);
    Controller.ButtonL2.pressed(reverseIntake);
    Controller.ButtonB.pressed(slowIntakeControl);
    Controller.ButtonB.released(fastIntakeControl);
    Controller.ButtonX.pressed(toggleIntakeOnly);

    Controller.Screen.clearScreen();
    // User control code here, inside the loop
    while (1)
    {
        double forward = Controller.Axis3.position();
        double lateral = Controller.Axis1.position();
        if (fabs(lateral) > 5)
        {
            lateral *= .8;
        }
        double leftSpeed = forward + lateral;
        double rightSpeed = forward - lateral;

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

int redirect()
{
    while (true)
    {
        if (conveyorMotor.torque() >= 0.35 && intakeSystemStatus == MotorStatus::forward)
        {
            wait(1000, msec);
            if (conveyorMotor.torque() >= 0.35)
            {
                conveyorMotor.spin(vex::reverse);
                wait(300, msec);
                conveyorMotor.spin(vex::forward);
            }
        }
        wait(50, msec);
    }
    return 0;
}

void colorSortFunc()
{
    if (colorSensor.hue() < 80) // red
    {
    }
    else if (colorSensor.hue() > 80) // blue
    {
        return;
    }
    wait(50, msec);
    conveyorMotor.stop();
    wait(500, msec);
    conveyorMotor.spin(vex::forward);
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

    vex::task armTask(armMovementTask);
    // vex::task redirectTask(redirect);

    // colorSensor.setLightPower(100, percent);
    // colorSensor.objectDetected(colorSortFunc);
    sideSelection();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
