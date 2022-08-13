#ifndef LANDING_H
#define LANDING_H

#include <ros/ros.h>

#define PRECISION(x)    round(x * 100) / 100
#define PI              3.14159265
#define YAW_ANGLE(x)    (x * (180 / PI))
#define RATODE(x)       (x / (180 / PI))

#define MAX_ALTITUDE    20
#define DISTANCE        0.7
#define ALTITUDE_CHANGE_ANGLE   15
#define ALTITUDE_CHANGE_METHOD  4
#define ERROR_ACCEPTANCE_DEGREES_30     30
#define ERROR_ACCEPTANCE_DEGREES_20     20
#define ERROR_ACCEPTANCE_DEGREES_15     15
#define ERROR_ACCEPTANCE_DEGREES_10     10
        
#define ERROR_ACCEPTANCE_YAW_DEGREES    10

#define INCREASE_ALTITUDE_NOT_DETECT    1

#define MAX_REPEAT_DETECT     4

#define TIME_NOW                ros::Time::now()
#define TIME_DURATION(x)        ros::Duration(x)
// #define LOG_INFO                1

#define STABLENUMBER        2

#endif /* LANDING_H */