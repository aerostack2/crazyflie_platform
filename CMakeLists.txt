cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME crazyflie_platform)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
#########################

# Add the crazyflie_cpp submodule
add_subdirectory(crazyflie_cpp)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# Faltal los packages del crazyflie_cpp


set(HEADER_HPP_FILES
  include/crazyflie_platform.hpp
)

include_directories(
  include/${PROJECT_NAME}
  crazyflie_cpp/include/crazyflie_cpp
)


add_executable(${PROJECT_NAME}_node src/crazyflie_node.cpp src/crazyflie_platform.cpp)
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_node 
  crazyflie_cpp
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




install(TARGETS
${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
