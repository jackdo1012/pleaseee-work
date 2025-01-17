#include <vex.h>

vex::drivetrain Drivetrain = vex::drivetrain(leftMotors, rightMotors);

void Odom::setInitPos(double xPos, double yPos, double orientation, double verticalOdoPos, double horizontalOdoPos)
{
    this->xPos = xPos;
    this->yPos = yPos;
    this->orientation = orientation;
    this->verticalOdoPos = verticalOdoPos;
    this->horizontalOdoPos = horizontalOdoPos;
}

// inches
void Odom::update(double verticalOdoPos, double horizontalOdoPos, double orientation)
{
    double verticalTrackingDistance = 1.084161;
    double horizontalTrackingDistance = 1.3358615;
    double wheelCircumference = 8.66142;

    double deltaVertical = verticalOdoPos - this->verticalOdoPos;
    double deltaHorizontal = horizontalOdoPos - this->horizontalOdoPos;
    this->verticalOdoPos = verticalOdoPos;
    this->horizontalOdoPos = horizontalOdoPos;
    double orientationRadian = toRadian(orientation);
    double prevOrientationRadian = toRadian(this->orientation);
    double deltaOrientation = orientationRadian - prevOrientationRadian;
    this->orientation = orientation;

    double verticalWheelMoved = deltaVertical * wheelCircumference;
    double horizontalWheelMoved = deltaHorizontal * wheelCircumference;

    double relDeltaFwd, relDeltaStr;
    if (deltaOrientation == 0)
    {
        relDeltaFwd = deltaVertical;
        relDeltaStr = deltaHorizontal;
    }
    else
    {
        relDeltaFwd =
            2 * (verticalWheelMoved / deltaOrientation + verticalTrackingDistance) * sin(deltaOrientation / 2);
        relDeltaStr =
            2 * (horizontalWheelMoved / deltaOrientation - horizontalTrackingDistance) * sin(deltaOrientation / 2);
    }
    this->xPos += relDeltaFwd * cos(orientationRadian) - relDeltaStr * sin(orientationRadian);
    this->yPos += relDeltaStr * cos(orientationRadian) + relDeltaFwd * sin(orientationRadian);
}
