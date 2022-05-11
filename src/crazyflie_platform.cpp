#include "crazyflie_platform.hpp"
#include <iostream>
#include "as2_core/core_functions.hpp"

CrazyfliePlatform::CrazyfliePlatform() : as2::AerialPlatform()
{
    RCLCPP_INFO(this->get_logger(), "Init");
    this->configureSensors();
}
void CrazyfliePlatform::configureSensors(){
    RCLCPP_INFO(this->get_logger(), "Configure Sensors");
}

bool CrazyfliePlatform::ownSendCommand(){
    return true;
}
bool CrazyfliePlatform::ownSetArmingState(bool state){
    return true;
}
bool CrazyfliePlatform::ownSetOffboardControl(bool offboard){
    return true;
}
bool CrazyfliePlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg){
    return true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CrazyfliePlatform>();
  node->preset_loop_frequency(300);
  as2::spinLoop(node);  
  RCLCPP_INFO(node->get_logger(), "bye");
  

  rclcpp::shutdown();
  return 0;
}