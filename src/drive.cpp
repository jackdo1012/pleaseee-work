#include <vex.h>

// Drive::Drive(vex::motor_group leftMotors, vex::motor_group rightMotors, int inertialPort, int verticalOdoPort,
//              int horizontalOdoPort, int leftBackMotorPort, int leftFrontMotorPort, int rightBackMotorPort,
//              int rightFrontMotorPort)
//     : Inertial(vex::inertial(inertialPort)), verticalOdo(vex::rotation(verticalOdoPort, false)),
//       horizontalOdo(vex::rotation(horizontalOdoPort, true)), leftBackMotor(vex::motor(leftBackMotorPort, true)),
//       leftFrontMotor(vex::motor(leftFrontMotorPort, true)), rightBackMotor(vex::motor(rightBackMotorPort, true)),
//       rightFrontMotor(vex::motor(rightFrontMotorPort, true))
// {
//     this->leftMotors = leftMotors;
//     this->rightMotors = rightMotors;
//     this->Inertial = vex::inertial(inertialPort);
//     this->verticalOdo = vex::rotation(verticalOdoPort, false);
//     this->horizontalOdo = vex::rotation(horizontalOdoPort, true);
//     this->leftBackMotor = vex::motor(leftBackMotorPort, true);
//     this->leftFrontMotor = vex::motor(leftFrontMotorPort, true);
//     this->rightBackMotor = vex::motor(rightBackMotorPort, true);
//     this->rightFrontMotor = vex::motor(rightFrontMotorPort, true);
//     this->drivePID = PID(0.0, 0.0, 0.0, 1.5);
//     this->turnPID = PID(0.0, 0.0, 0.0, 1);
//     this->swingPID = PID(0.0, 0.0, 0.0, 1);
// }
Drive::Drive(int inertialPort, int verticalOdoPort, int horizontalOdoPort, vex::motor_group leftMotors,
             vex::motor_group rightMotors)
    : Inertial(vex::inertial(vex::PORT11, vex::turnType::left)), verticalOdo(vex::rotation(vex::PORT13, false)),
      horizontalOdo(vex::rotation(vex::PORT12, true)), leftMotors(leftMotors), rightMotors(rightMotors)
{
    this->drivePID = PID(2.5, 0, 10, 1.5);
    this->turnPID = PID(0.7, .03, 3, 1);
    this->swingPID = PID(.3, .001, 2, 1);
    this->driveMaxVolt = 10;
    this->turnMaxVolt = 12;
    this->swingMaxVolt = 12;
}

void Drive::drive(double leftVolt, double rightVolt)
{
    rightMotors.spin(vex::forward, rightVolt, vex::voltageUnits::volt);
    leftMotors.spin(vex::forward, leftVolt, vex::voltageUnits::volt);
}

void Drive::positionTrack()
{
    while (true)
    {
        odom.update(this->verticalOdo.position(vex::rotationUnits::rev),
                    this->horizontalOdo.position(vex::rotationUnits::rev),
                    formatAngle360(this->Inertial.rotation(vex::rotationUnits::rev) * 360));
        vex::task::sleep(5);
    }
}

int Drive::positionTrackTask()
{
    chassis.positionTrack();
    return 0;
}

void Drive::setInitPos(double xPos, double yPos, double orientation)
{
    this->Inertial.setHeading(orientation, vex::rotationUnits::deg);
    this->Inertial.setRotation(orientation / 360, vex::rotationUnits::rev);

    odom.setInitPos(xPos, yPos, orientation, verticalOdo.position(vex::rotationUnits::rev),
                    horizontalOdo.position(vex::rotationUnits::rev));
    vex::task odomTask(positionTrackTask);
}

void Drive::driveToPoint(double x, double y, double heading, double maxTime)
{
    this->drivePID.start(hypot(x - odom.xPos, y - odom.yPos), maxTime);
    this->turnPID.start(heading - formatAngle360(odom.orientation), maxTime);
    while (!this->drivePID.isDone() || !this->turnPID.isDone())
    {
        double driveErr = hypot(x - odom.xPos, y - odom.yPos);
        double turnErr =
            formatAngle180(toDegree(atan2(y - odom.yPos, x - odom.xPos)) - formatAngle360(odom.orientation));
        double driveOutput = drivePID.execute(driveErr);

        double turnScaleFactor = cos(toRadian(turnErr));
        driveOutput *= turnScaleFactor;
        turnErr = formatAngle90(turnErr);
        double turnOutput = turnPID.execute(turnErr);

        if (driveErr < drivePID.tolerance)
        {
            turnOutput = 0;
        }

        driveOutput =
            clamp(driveOutput, -1 * fabs(turnScaleFactor) * this->driveMaxVolt, fabs(turnScaleFactor) * driveMaxVolt);
        turnOutput = clamp(turnOutput, -1 * turnMaxVolt, turnMaxVolt);

        double leftVolt = driveOutput - turnOutput;
        double rightVolt = driveOutput + turnOutput;

        if (leftVolt > this->driveMaxVolt || rightVolt > this->driveMaxVolt)
        {
            double ratio = leftVolt / rightVolt;
            if (leftVolt > rightVolt)
            {
                leftVolt = this->driveMaxVolt;
                rightVolt = this->driveMaxVolt * ratio;
            }
            else
            {
                rightVolt = this->driveMaxVolt;
                leftVolt = this->driveMaxVolt / ratio;
            }
        }
        this->drive(leftVolt, rightVolt);

        vex::task::sleep(5);
    }
}

void Drive::turnToHeading(double heading, double maxTime)
{
    this->turnPID.start(formatAngle180(heading - formatAngle360(odom.orientation)), maxTime);
    while (!this->turnPID.isDone())
    {
        double turnOutput =
            clamp(-1 * this->turnMaxVolt, turnPID.execute(formatAngle180(heading - formatAngle360(odom.orientation))),
                  this->turnMaxVolt);
        this->drive(-turnOutput, turnOutput);
        vex::task::sleep(10);
    }
}

void Drive::leftSwing(double heading, double maxTime)
{
    this->swingPID.start(formatAngle180(heading - formatAngle360(odom.orientation)), maxTime);
    while (!this->swingPID.isDone())
    {
        double swingOutput =
            clamp(-1 * this->swingMaxVolt, swingPID.execute(formatAngle180(heading - formatAngle360(odom.orientation))),
                  this->swingMaxVolt);
        this->leftMotors.spin(vex::reverse, swingOutput, vex::voltageUnits::volt);
        this->rightMotors.stop(vex::brakeType::hold);
        vex::task::sleep(10);
    }
}
void Drive::rightSwing(double heading, double maxTime)
{
    this->swingPID.start(formatAngle180(heading - formatAngle360(odom.orientation)), maxTime);
    while (!this->swingPID.isDone())
    {
        double swingOutput =
            clamp(-1 * this->swingMaxVolt, swingPID.execute(formatAngle180(heading - formatAngle360(odom.orientation))),
                  this->swingMaxVolt);
        this->rightMotors.spin(vex::forward, swingOutput, vex::voltageUnits::volt);
        this->leftMotors.stop(vex::brakeType::hold);
        vex::task::sleep(10);
    }
}