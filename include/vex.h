#pragma once
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "devices.h"
#include "util.h"
#include "odom.h"
#include "pid.h"
#include "drive.h"

class Drive;
extern Drive chassis;

#define waitUntil(condition)                                                                                           \
    do                                                                                                                 \
    {                                                                                                                  \
        wait(5, msec);                                                                                                 \
    } while (!(condition))

#define repeat(iterations) for (int iterator = 0; iterator < iterations; iterator++)