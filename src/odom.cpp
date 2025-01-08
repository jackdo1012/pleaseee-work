#include <vex.h>

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

    double relDeltaFwd, relDeltaStr;

    if (deltaOrientation == 0)
    {
        relDeltaFwd = verticalOdoPos;
        relDeltaStr = horizontalOdoPos;
    }
    else
    {
        relDeltaFwd = 2 * (deltaVertical * wheelCircumference / deltaOrientation + verticalTrackingDistance) *
                      sin(deltaOrientation / 2);
        relDeltaStr = 2 * (deltaHorizontal * wheelCircumference / deltaOrientation - horizontalTrackingDistance) *
                      sin(deltaOrientation / 2);
    }
    this->yPos += relDeltaFwd * cos(deltaOrientation) - relDeltaStr * sin(deltaOrientation);
    this->xPos -= relDeltaStr * cos(deltaOrientation) + relDeltaFwd * sin(deltaOrientation);
}
