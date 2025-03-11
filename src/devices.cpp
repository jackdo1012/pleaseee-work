#include "vex.h"

using namespace vex;

vex::brain Brain = vex::brain();
vex::controller Controller = vex::controller();
// motors and piston
vex::motor intakeMotor = vex::motor(PORT16, ratio6_1, true);
vex::motor conveyorMotor = vex::motor(PORT20, ratio6_1, true);
vex::motor leftArmMotor = vex::motor(PORT10, false);
vex::motor rightArmMotor = vex::motor(PORT9, true);
vex::digital_out clampCylinder = vex::digital_out(Brain.ThreeWirePort.A);
vex::digital_out doinkerCylinder = vex::digital_out(Brain.ThreeWirePort.H);
vex::digital_out armCylinder = vex::digital_out(Brain.ThreeWirePort.G);
// sensors
vex::inertial Inertial = vex::inertial(PORT18);
vex::optical colorSensor = vex::optical(PORT5);
// drivetrain
vex::motor leftBackMotor = vex::motor(PORT12, ratio6_1, true);
vex::motor leftFrontMotor = vex::motor(PORT14, ratio6_1, true);
vex::motor rightBackMotor = vex::motor(PORT13, ratio6_1, false);
vex::motor rightFrontMotor = vex::motor(PORT15, ratio6_1, false);
vex::motor_group leftMotors = vex::motor_group(leftBackMotor, leftFrontMotor);
vex::motor_group rightMotors = vex::motor_group(rightBackMotor, rightFrontMotor);