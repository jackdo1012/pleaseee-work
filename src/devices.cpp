#include "vex.h"

using namespace vex;

vex::controller Controller = vex::controller();
vex::brain Brain = vex::brain();
// motors and piston
vex::motor intakeMotor = vex::motor(PORT5, ratio6_1, true);
vex::motor conveyorMotor = vex::motor(PORT6, ratio6_1, true);
vex::motor leftArmMotor = vex::motor(PORT7);
vex::motor rightArmMotor = vex::motor(PORT8);
vex::digital_out clampCylinder = vex::digital_out(Brain.ThreeWirePort.A);
vex::digital_out powerCylinder = vex::digital_out(Brain.ThreeWirePort.B);
// sensors
vex::rotation verticalOdo = vex::rotation(PORT11);
vex::rotation horizontalOdo = vex::rotation(PORT12);
vex::rotation armSensor = vex::rotation(PORT10);
vex::inertial Inertial = vex::inertial(PORT9);
// drivetrain
vex::motor leftBackMotor = vex::motor(PORT1, true);
vex::motor leftFrontMotor = vex::motor(PORT2, true);
vex::motor rightBackMotor = vex::motor(PORT3, true);
vex::motor rightFrontMotor = vex::motor(PORT4, true);
vex::motor_group leftMotors = vex::motor_group(leftBackMotor, leftFrontMotor);
vex::motor_group rightMotors = vex::motor_group(rightBackMotor, rightFrontMotor);