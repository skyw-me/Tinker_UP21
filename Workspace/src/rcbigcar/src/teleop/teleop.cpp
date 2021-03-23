#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>

#include "rcbigcar/TeleopConfig.h"


const double kWatchdogTimeout = 1.0;

// dynamic reconfiguration
rcbigcar::TeleopConfig config;

// comm
ros::Publisher twist_pub;
ros::Subscriber joy_sub;

// twist
geometry_msgs::Twist twist;

// watchdog
ros::Time watchdog_last_time = ros::Time(0);

void callback_joy(const sensor_msgs::Joy::ConstPtr &joy)
{
    // watchdog
    watchdog_last_time = ros::Time::now();

    // set speed
    twist.angular.z = joy->axes[0] * config.MaxW;

    twist.linear.x = joy->axes[4] * config.MaxX;
    twist.linear.y = joy->axes[3] * config.MaxY;
}

void callback_param( rcbigcar::TeleopConfig &_config, uint32_t level ) {
    config = _config;
}

void zero_velocity() {
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
}

void update_watchdog() {
    if ((ros::Time::now() - watchdog_last_time).toSec() > kWatchdogTimeout) {
        zero_velocity();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop");

    ros::NodeHandle nh;

    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &callback_joy);

    // init twist
    zero_velocity();

    // dynamic reconfigure
    dynamic_reconfigure::Server<rcbigcar::TeleopConfig> param_server;
    param_server.setCallback(callback_param);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        // update watchdog
        update_watchdog();

        // publish
        twist_pub.publish(twist);

        // loop
        ros::spinOnce();
        loop_rate.sleep();
    }
}
