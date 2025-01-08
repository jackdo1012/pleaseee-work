#pragma once
#include <vex.h>

class Drive
{
  private:
    vex::motor leftBackMotor = leftBackMotor;
    vex::motor leftFrontMotor = leftFrontMotor;
    vex::motor rightBackMotor = rightBackMotor;
    vex::motor rightFrontMotor = rightFrontMotor;
    vex::motor_group leftMotors = leftMotors;
    vex::motor_group rightMotors = rightMotors;
    vex::inertial Inertial;
    PID drivePID;
    int driveMaxVolt;
    PID turnPID;
    int turnMaxVolt;
    PID swingPID;
    int swingMaxVolt;
    vex::rotation verticalOdo;
    vex::rotation horizontalOdo;

  public:
    Odom odom;
    Drive(int inertialPort, int verticalOdoPort, int horizontalOdoPort);
    // Drive(vex::motor_group leftMotors, vex::motor_group rightMotors, int inertialPort, int verticalOdoPort,
    //       int horizontalOdoPort, int leftBackMotorPort, int leftFrontMotorPort, int rightBackMotorPort,
    //       int rightFrontMotorPort);
    void drive(double leftVolt, double rightVolt);
    void turnToHeading(double heading);
    void driveToPoint(double x, double y, double heading);
    void positionTrack();
    static int positionTrackTask();
    void setInitPos(double xPos, double yPos, double orientation);
    void leftSwing(double heading);
    void rightSwing(double heading);
};