#include <vex.h>

Drive::Drive(int inertialPort, int verticalOdoPort, int horizontalOdoPort, vex::motor_group leftMotors,
             vex::motor_group rightMotors)
    : Inertial(vex::inertial(vex::PORT18, vex::turnType::left)), verticalOdo(vex::rotation(vex::PORT17, false)),
      horizontalOdo(vex::rotation(vex::PORT19, true)), leftMotors(leftMotors), rightMotors(rightMotors)
{
    this->drivePID = PID(2.1, 0, 20, 1);
    this->headingPID = PID(1, 0, 15, 1);
    this->turnPID = PID(0.4, 0, 6, 1);
    this->swingPID = PID(0.3, .001, 2, 1);
    this->defaultDrivePid();
    this->defaultTurnPid();
    this->defaultSwingPid();
    this->defaultHeadingPid();

    this->driveMaxVolt = 10;
    this->headingMaxVolt = 6;
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
                    this->horizontalOdo.position(vex::rotationUnits::rev), formatAngle360(this->Inertial.heading()));
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

void Drive::driveToPoint(double x, double y, double maxTime)
{
    this->driveToPoint(x, y, maxTime, 0);
}
void Drive::driveToPoint(double x, double y, double maxTime, double minSettlingTime)
{
    if (minSettlingTime == 0)
    {
        minSettlingTime = 400;
    }

    double absoluteHeading = toDegree(atan2(y - odom.yPos, x - odom.xPos));
    this->drivePID.start(hypot(x - odom.xPos, y - odom.yPos), maxTime, minSettlingTime);
    this->headingPID.start(absoluteHeading - odom.orientation, maxTime, minSettlingTime);

    bool lineSettled = false;
    bool prevLineSettled = isLineCrossed(x, y, odom.xPos, odom.yPos, absoluteHeading);

    while (!this->drivePID.isDone())
    {
        lineSettled = isLineCrossed(x, y, odom.xPos, odom.yPos, absoluteHeading);
        if (lineSettled && !prevLineSettled)
        {
            break;
        }
        prevLineSettled = lineSettled;

        double driveErr = hypot(x - odom.xPos, y - odom.yPos);
        double turnErr =
            formatAngle180(toDegree(atan2(y - odom.yPos, x - odom.xPos)) - formatAngle360(odom.orientation));
        double driveOutput = drivePID.execute(driveErr);

        double turnScaleFactor = cos(toRadian(turnErr));
        driveOutput *= turnScaleFactor;
        turnErr = formatAngle90(turnErr);
        double turnOutput = headingPID.execute(turnErr);

        if (driveErr < drivePID.tolerance)
        {
            turnOutput = 0;
        }
        driveOutput =
            clamp(-1 * fabs(turnScaleFactor) * this->driveMaxVolt, driveOutput, fabs(turnScaleFactor) * driveMaxVolt);
        turnOutput = clamp(-1 * this->headingMaxVolt, turnOutput, this->headingMaxVolt);

        this->drive(leftVoltScaling(driveOutput, turnOutput), rightVoltScaling(driveOutput, turnOutput));

        vex::task::sleep(5);
    }
    this->stop();
}

void Drive::driveToPosition(double x, double y, double heading, double maxTime, bool forward)
{
    this->driveToPosition(x, y, heading, maxTime, forward, 0);
};

void Drive::driveToPosition(double x, double y, double heading, double maxTime, bool forward, double minSettlingTime)
{
    if (minSettlingTime == 0)
    {
        minSettlingTime = 400;
    }

    this->drivePID.start(hypot(x - odom.xPos, y - odom.yPos), maxTime, minSettlingTime);
    this->headingPID.start(toDegree(atan2(x - odom.xPos, y - odom.yPos)) - formatAngle360(odom.orientation), maxTime,
                           minSettlingTime);

    bool centerLineCrossed = false;
    bool centerLineSide = isLineCrossed(x, y, odom.xPos, odom.yPos, heading + 90);
    bool prevCenterLineSide = centerLineSide;

    while (!this->drivePID.isDone())
    {
        centerLineSide = isLineCrossed(x, y, odom.xPos, odom.yPos, heading + 90);
        if (centerLineSide != prevCenterLineSide)
        {
            centerLineCrossed = true;
        }

        double distance = hypot(x - odom.xPos, y - odom.yPos);

        double carrotX = x - distance * sin(toRadian(heading)) * this->boomerangLead;
        double carrotY = y - distance * cos(toRadian(heading)) * this->boomerangLead;

        double driveErr = hypot(carrotX - odom.xPos, carrotY - odom.yPos);
        double turnErr = formatAngle180(toDegree(atan2(carrotX - odom.xPos, carrotY - odom.yPos)) -
                                        formatAngle360(odom.orientation));

        if (centerLineCrossed)
        {
            driveErr = distance;
            turnErr = formatAngle180(heading - formatAngle360(odom.orientation));
        }

        double driveOutput = drivePID.execute(driveErr);
        double turnScaleFactor = cos(toRadian(turnErr));
        driveOutput *= turnScaleFactor;
        turnErr = formatAngle90(turnErr);
        double turnOutput = headingPID.execute(turnErr);

        driveOutput =
            clamp(-1 * fabs(turnScaleFactor) * this->driveMaxVolt, driveOutput, fabs(turnScaleFactor) * driveMaxVolt);
        turnOutput = clamp(-1 * this->headingMaxVolt, turnOutput, this->headingMaxVolt);

        this->drive(leftVoltScaling(driveOutput, turnOutput), rightVoltScaling(driveOutput, turnOutput));

        vex::task::sleep(5);
    }
    this->stop();
};

void Drive::turnToHeading(double heading, double maxTime)
{
    this->turnToHeading(heading, maxTime, 0);
}
void Drive::turnToHeading(double heading, double maxTime, double minSettlingTime)
{
    if (minSettlingTime == 0)
    {
        minSettlingTime = 400;
    }
    this->turnPID.start(formatAngle180(heading - formatAngle360(odom.orientation)), maxTime, minSettlingTime);
    while (!this->turnPID.isDone())
    {
        double turnOutput = turnPID.execute(formatAngle180(heading - formatAngle360(odom.orientation)));
        turnOutput = clamp(-1 * this->turnMaxVolt, turnOutput, this->turnMaxVolt);
        this->drive(-turnOutput, turnOutput);
        vex::task::sleep(5);
    }
}

void Drive::leftSwing(double heading, double maxTime)
{
    this->leftSwing(heading, maxTime, 0);
}
void Drive::leftSwing(double heading, double maxTime, double minSettlingTime)
{
    if (minSettlingTime == 0)
    {
        minSettlingTime = 400;
    }
    this->swingPID.start(formatAngle180(heading - formatAngle360(odom.orientation)), maxTime, minSettlingTime);
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
    this->rightSwing(heading, maxTime, 0);
}
void Drive::rightSwing(double heading, double maxTime, double minSettlingTime)
{
    if (minSettlingTime == 0)
    {
        minSettlingTime = 400;
    }
    this->swingPID.start(formatAngle180(heading - formatAngle360(odom.orientation)), maxTime, minSettlingTime);
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

void Drive::changeDrivePid(double kP, double kI, double kD)
{
    this->drivePID.changeConst(kP, kI, kD);
}
void Drive::changeDrivePid(double kP, double kI, double kD, double tolerance)

{
    this->drivePID.changeConst(kP, kI, kD, tolerance);
}

void Drive::changeTurnPid(double kP, double kI, double kD)
{
    this->turnPID.changeConst(kP, kI, kD);
}

void Drive::changeTurnPid(double kP, double kI, double kD, double tolerance)
{
    this->turnPID.changeConst(kP, kI, kD, tolerance);
}

void Drive::changeSwingPid(double kP, double kI, double kD)
{
    this->turnPID.changeConst(kP, kI, kD);
}

void Drive::changeSwingPid(double kP, double kI, double kD, double tolerance)
{
    this->turnPID.changeConst(kP, kI, kD, tolerance);
}

void Drive::changeBoomerangLead(double lead)
{
    this->boomerangLead = lead;
}

void Drive::defaultDrivePid()
{
    this->drivePID.changeConst(2.1, 0, 20, 1);
}

void Drive::defaultHeadingPid()
{
    this->drivePID.changeConst(1, 0, 15, 1);
}

void Drive::defaultTurnPid()
{
    this->drivePID.changeConst(0.4, 0, 6, 1);
}

void Drive::defaultSwingPid()
{
    this->drivePID.changeConst(0.3, .001, 2, 1);
}

void Drive::stop()
{
    this->drive(0, 0);
}
