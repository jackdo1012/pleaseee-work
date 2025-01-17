#include <vex.h>

PID::PID()
{
    this->kP = 0;
    this->kI = 0;
    this->kD = 0;
}

PID::PID(double kP, double kI, double kD, double tolerance)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->tolerance = tolerance;
}

// How PID work: The execute func will be called every ... ms with param err
// - Error: desired value - current value (e.g: you want to move 10mm and
// current position is 0mm -> err = 10mm)
// - Previous error: error of the previous func call
// - Total error: total err of every func call
// - (P)ropotional: err * P. The more error -> the higher speed
// - (I)ntergral: integral * I. This value will speed up the robot when the
// error adds up
// - (D)erivative: err - prevErr * D. This value will slow the robot down as
// the robot approach the target
// - The output will be the sum of those 3
double PID::execute(double err)
{
    this->integral += err;

    // If error cross 0, reset integral so the robot won't oscillate
    if ((err > 0 && prevErr < 0) || (err < 0 && prevErr > 0))
    {
        this->integral = 0;
    }

    double output = this->kP * err + this->kI * this->integral + this->kD * (err - prevErr);

    this->prevErr = err;

    if (std::abs(err) <= this->tolerance)
    {
        this->settlingTime += 10;
    }
    else
    {
        this->settlingTime = 0;
    }

    this->runningTime += 10;

    return output;
}

void PID::start(double err, double maxTime)
{
    this->prevErr = 0;
    this->integral = 0;
    this->settlingTime = 0;
    this->runningTime = 0;
    this->maxTime = maxTime;
    this->execute(err);
}

bool PID::isDone()
{
    if (this->runningTime > this->maxTime && this->maxTime != 0)
    {
        return true;
    }
    if (this->settlingTime >= this->minSettlingTime)
    {
        return true;
    }
    return false;
}