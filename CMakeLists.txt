cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME crazyflie_platform)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(Eigen3 REQUIRED)
# Faltal los packages del crazyflie_cpp


set(HEADER_HPP_FILES
  include/crazyflie_platform.hpp
)

include_directories(
  include/${PROJECT_NAME}
  crazyflie_cpp/include/crazyflie_cpp
  ${EIGEN3_INCLUDE_DIRS}
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
  Eigen3
)
target_link_libraries(${PROJECT_NAME}_node 
  crazyflie_cpp
)



if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
