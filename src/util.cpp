#define M_PI 3.14159265358979323846

double toRadian(double degrees)
{
    return degrees * M_PI / 180;
}

double toDegree(double radians)
{
    return radians * 180 / M_PI;
}

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

double formatAngle180(double angle)
{
    while (angle <= -180 || angle > 180)
    {
        if (angle <= -180)
        {
            angle += 360;
        }
        if (angle > 180)
        {
            angle -= 360;
        }
    }
    return angle;
}

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

double clamp(double minX, double x, double maxX)
{
    return (minX > ((x < maxX) ? x : maxX) ? minX : ((x < maxX) ? x : maxX));
}