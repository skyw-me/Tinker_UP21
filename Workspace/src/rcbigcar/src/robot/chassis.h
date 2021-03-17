#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "config.h"
#include "hardware.h"
#include "motor.h"

#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include "rcbigcar/ChassisConfig.h"

class Chassis
{
public:
  Chassis();
  ~Chassis();

  /* Update Funcs */
  void update();
  void UpdateOdometry();
  void UpdateWatchdog();
  void UpdateDebug();

  /* Publish Funcs */
  void PublishPosition();

  /* Callback Funcs */
  void CallbackDynamicParam(rcbigcar::ChassisConfig& config, uint32_t level);
  void CallbackVelocity(const geometry_msgs::Twist::ConstPtr& twist);

private:
  /*
   * Config
  */
  bool Config_IsDebug;

  double Dyn_Config_MaxVel;
  /*
   * Handles
   */
  ros::Subscriber twist_sub;
  tf::TransformBroadcaster tf_pos_pub;

  /*
  * Motor
  */
  Motor* motors[4];
  ros::Time motorWatchdog;

  /*
  * Odometry
  */
  double x, y, theta;
  double last_position[4];

  /*
  * Debug
  */
  ros::Publisher dbg_spd_setpoint_pub;
  ros::Publisher dbg_spd_real_pub;
};

#endif