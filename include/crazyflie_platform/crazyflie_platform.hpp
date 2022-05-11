#ifndef CRAZYFLIE_PLATFORM_HPP_
#define CRAZYFLIE_PLATFORM_HPP_

#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "rclcpp/rclcpp.hpp"

class CrazyfliePlatform : public as2::AerialPlatform
{
    public:
    CrazyfliePlatform();

    void configureSensors();

    bool ownSetArmingState(bool state);
    bool ownSetOffboardControl(bool offboard);
    bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg);
    bool ownSendCommand();
};

#endif