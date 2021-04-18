#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>

#include "rcbigcar/TeleopConfig.h"
#include "ramp.h"


const double kWatchdogTimeout = 1.0;
const double kDeactivationTimeout = 30.0;

// dynamic reconfiguration
rcbigcar::TeleopConfig config;

// comm
ros::Publisher twist_pub;
ros::Subscriber joy_sub;

// ramp
RampFilter ramp_filters[3];

// twist
geometry_msgs::Twist twist_target;

bool twist_active = false;

// watchdog
ros::Time watchdog_last_time = ros::Time(0);

void callback_joy(const sensor_msgs::Joy::ConstPtr &joy)
{
    // watchdog
    watchdog_last_time = ros::Time::now();

    // set speed
    // Logitech F710 Gamepad
    twist_target.angular.z = joy->axes[0] * config.MaxW;
    twist_target.linear.x = joy->axes[3] * config.MaxX;
    twist_target.linear.y = joy->axes[2] * config.MaxY;

    // activate
    twist_active = true;
}

void callback_param( rcbigcar::TeleopConfig &_config, uint32_t level ) {
    config = _config;
}

void zero_velocity(geometry_msgs::Twist &t) {
    t.angular.x = 0;
    t.angular.y = 0;
    t.angular.z = 0;

    t.linear.x = 0;
    t.linear.y = 0;
    t.linear.z = 0;
}

void update_watchdog() {
    // stop robot
    if ((ros::Time::now() - watchdog_last_time).toSec() > kWatchdogTimeout) {
        zero_velocity(twist_target);
    }

    // deactivate
    if ((ros::Time::now() - watchdog_last_time).toSec() > kDeactivationTimeout) {
        twist_active = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop");

    ros::NodeHandle nh;

    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &callback_joy);

    // init twist
    zero_velocity(twist_target);

    // dynamic reconfigure
    dynamic_reconfigure::Server<rcbigcar::TeleopConfig> param_server;
    param_server.setCallback(callback_param);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        // update watchdog
        update_watchdog();

        // publish
        if (twist_active) {
            // current twist
            geometry_msgs::Twist twist;
            zero_velocity(twist);

            // update twist
            twist.angular.z = ramp_filters[0].filter(twist_target.angular.z, config.MaxW, config.MaxAccW);
            twist.linear.x = ramp_filters[1].filter(twist_target.angular.x, config.MaxX, config.MaxAccX);
            twist.linear.y = ramp_filters[2].filter(twist_target.angular.y, config.MaxY, config.MaxAccY);

            // publish
            twist_pub.publish(twist);
        }

        // loop
        ros::spinOnce();
        loop_rate.sleep();
    }
}
