#define M_PI 3.14159265358979323846
#include "vex.h"

double toRadian(double degrees)
{
    return degrees * M_PI / 180;
}

double toDegree(double radians)
{
    return radians * 180 / M_PI;
}

// format angle to values between 0 and 360deg
double formatAngle360(double angle)
{
    while (angle <= 0 || angle > 360)
    {
        if (angle < 0)
        {
            angle += 360;
        }
        if (angle >= 360)
        {
            angle -= 360;
        }
    }
    return angle;
}

// format angle to values between -180 and 180deg
double formatAngle180(double angle)
{
    while (!(angle >= -180 && angle < 180))
    {
        if (angle < -180)
        {
            angle += 360;
        }
        if (angle >= 180)
        {
            angle -= 360;
        }
    }
    return angle;
}

// format angle to values between -90 and 90deg
// if angle is not between -90 and 90deg, return the angle +- 180deg (opposite angle)
double formatAngle90(double angle)
{
    while (angle <= -90 || angle > 90)
    {
        if (angle <= -90)
        {
            angle += 180;
        }
        if (angle > 90)
        {
            angle -= 180;
        }
    }
    return angle;
}

double clamp(double minx, double x, double maxx)
{
    return (minx > ((x < maxx) ? x : maxx) ? minx : ((x < maxx) ? x : maxx));
}

double isLineCrossed(double targetX, double targetY, double currentX, double currentY, double heading)
{
    return ((targetY - currentY) * cos(toRadian(heading)) <= -(targetX - currentX) * sin(toRadian(heading)));
}

double leftVoltScaling(double drive, double turn)
{
    double ratio = std::fmax(std::fabs(drive + turn), std::fabs(drive - turn)) / 12;
    if (ratio > 1)
    {
        return (drive - turn) / ratio;
    }
    return drive - turn;
}

double rightVoltScaling(double drive, double turn)
{
    double ratio = std::fmax(std::fabs(drive + turn), std::fabs(drive - turn)) / 12.0;
    if (ratio > 1)
    {
        return (drive + turn) / ratio;
    }
    return drive + turn;
}