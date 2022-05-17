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
#include <Crazyflie.h>

class CrazyfliePlatform : public as2::AerialPlatform
{
    public:
    CrazyfliePlatform();

    void configureSensors();

    bool ownSetArmingState(bool state);
    bool ownSetOffboardControl(bool offboard);
    bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg);
    bool ownSendCommand();

    // Crazyflie Functions
    void listVariables();
    void pingCB();
    void onLogIMU(uint32_t time_in_ms, std::vector<double>* values, void* /*userData*/);
    void onLogOdomOri(uint32_t time_in_ms, std::vector<double>* values, void* /*userData*/);
    void onLogOdomPos(uint32_t time_in_ms, std::vector<double>* values, void* /*userData*/);
    void updateOdom();

    private:
    std::shared_ptr<Crazyflie> cf_;
    rclcpp::TimerBase::SharedPtr ping_timer_;
    bool is_connected_;

    /*  --  SENSORS --  */


    // Odometry
    //using Odometry = as2::sensors::Sensor<nav_msgs::msg::Odometry>;
    std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odom_estimate_ptr_;
    double odom_buff_[10];
    std::function<void(uint32_t, std::vector<double>*, void*)> cb_odom_ori_;
    std::shared_ptr<LogBlockGeneric> odom_logBlock_ori_;
    bool ori_rec_;

    std::function<void(uint32_t, std::vector<double>*, void*)> cb_odom_pos_;
    std::shared_ptr<LogBlockGeneric> odom_logBlock_pos_;
    bool pos_rec_;



    // IMU
    std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
    double imu_buff_[6];
    std::function<void(uint32_t, std::vector<double>*, void*)> cb_imu_;
    std::shared_ptr<LogBlockGeneric> imu_logBlock_;

    // Opt FLow
    // TO DO
    
};

#endif