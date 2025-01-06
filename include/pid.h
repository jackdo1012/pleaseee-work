#pragma once
#include <vex.h>

class PID
{
  private:
    double kP;
    double kI;
    double kD;

    double prevErr = 0;
    double integral = 0;

  public:
    double settlingTime = 0;
    double minSettlingTime = 500;
    double tolerance = 1;
    PID();
    PID(double kP, double kI, double kD, double tolerance);
    double execute(double err);
    void start(double err);
    bool isDone();
};