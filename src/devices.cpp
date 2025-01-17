#include "vex.h"

using namespace vex;

vex::controller Controller = vex::controller();
vex::brain Brain = vex::brain();
// motors and piston
vex::motor intakeMotor = vex::motor(PORT7, ratio6_1, true);
vex::motor conveyorMotor = vex::motor(PORT19, ratio6_1, true);
vex::motor ptoMotor = vex::motor(PORT18, ratio6_1, true);
vex::motor leftArmMotor = vex::motor(PORT9, false);
vex::motor rightArmMotor = vex::motor(PORT10, true);
vex::digital_out clampCylinder = vex::digital_out(Brain.ThreeWirePort.A);
vex::digital_out powerCylinder = vex::digital_out(Brain.ThreeWirePort.B);
// sensors
vex::rotation verticalOdo = vex::rotation(PORT13);
vex::rotation horizontalOdo = vex::rotation(PORT12);
vex::rotation armSensor = vex::rotation(PORT8);
vex::inertial Inertial = vex::inertial(PORT11);
// drivetrain
vex::motor leftBackMotor = vex::motor(PORT3, ratio6_1, true);
vex::motor leftFrontMotor = vex::motor(PORT2, ratio6_1, true);
vex::motor rightBackMotor = vex::motor(PORT14, ratio6_1, false);
vex::motor rightFrontMotor = vex::motor(PORT16, ratio6_1, false);
vex::motor_group leftMotors = vex::motor_group(leftBackMotor, leftFrontMotor);
vex::motor_group rightMotors = vex::motor_group(rightBackMotor, rightFrontMotor);