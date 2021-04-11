#ifndef __RAMP_FILTER___
#define __RAMP_FILTER___


#include <ros/ros.h>

struct RampFilter
{
    ros::Time last_time = ros::Time(0);
    double last_value = 0;

    double limit_abs(double x, double max_abs)
    {
        double sign = (x < 0) ? -1 : 1;
        return sign * std::min(std::abs(x), std::abs(max_abs));
    }
    double filter(double set_value, double max_value, double accel)
    {
        double dt = (ros::Time::now() - last_time).toSec();
        if (!dt)
        {
            return set_value;
        }

        //limit input
        set_value = limit_abs(set_value, max_value);

        //ramp filter
        last_time = ros::Time::now();
        last_value += limit_abs(set_value - last_value, accel * dt);
        return last_value;
    }
};

#endif