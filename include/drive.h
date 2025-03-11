#pragma once
#include <vex.h>

class Drive
{
  private:
    vex::motor leftBackMotor = leftBackMotor;
    vex::motor leftFrontMotor = leftFrontMotor;
    vex::motor rightBackMotor = rightBackMotor;
    vex::motor rightFrontMotor = rightFrontMotor;
    vex::motor_group leftMotors;
    vex::motor_group rightMotors;
    vex::inertial Inertial;

    vex::rotation verticalOdo;
    vex::rotation horizontalOdo;

    PID drivePID;
    int driveMaxVolt;
    PID headingPID;
    int headingMaxVolt;
    PID turnPID;
    int turnMaxVolt;
    PID swingPID;
    int swingMaxVolt;

    double boomerangLead = 0.5;

  public:
    Odom odom;
    Drive(int inertialPort, int verticalOdoPort, int horizontalOdoPort, vex::motor_group leftMotors,
          vex::motor_group rightMotors);
    void drive(double leftVolt, double rightVolt);

    void turnToHeading(double heading, double maxTime);
    void turnToHeading(double heading, double maxTime, double minSettlingTime);

    void driveToPoint(double x, double y, double maxTime);
    void driveToPoint(double x, double y, double maxTime, double minSettlingTime);

    void driveToPosition(double x, double y, double heading, double maxTime, bool forward);
    void driveToPosition(double x, double y, double heading, double maxTime, bool forward, double minSettlingTime);

    void positionTrack();
    static int positionTrackTask();
    void setInitPos(double xPos, double yPos, double orientation);

    void leftSwing(double heading, double maxTime, double minSettlingTime);
    void leftSwing(double heading, double maxTime);

    void rightSwing(double heading, double maxTime, double minSettlingTime);
    void rightSwing(double heading, double maxTime);

    void changeDrivePid(double kP, double kI, double kD);
    void changeDrivePid(double kP, double kI, double kD, double tolerance);
    void changeTurnPid(double kP, double kI, double kD);
    void changeTurnPid(double kP, double kI, double kD, double tolerance);
    void changeSwingPid(double kP, double kI, double kD);
    void changeSwingPid(double kP, double kI, double kD, double tolerance);
    void changeBoomerangLead(double lead);

    void defaultDrivePid();
    void defaultHeadingPid();
    void defaultTurnPid();
    void defaultSwingPid();

    void stop();
};