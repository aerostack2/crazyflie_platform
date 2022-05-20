#include "crazyflie_platform.hpp"
#include <iostream>
#include "as2_core/core_functions.hpp"

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CrazyfliePlatform>();
    node->preset_loop_frequency(300);
    as2::spinLoop(node);  
    RCLCPP_INFO(node->get_logger(), "Ending!");


    rclcpp::shutdown();
    return 0;
}