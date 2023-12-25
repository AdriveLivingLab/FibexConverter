cmake_minimum_required(VERSION 3.5)

project(ixxat_gw_{{channel_name}} LANGUAGES CXX)

# set default C++ compiler options according to the recommendation by ROS: https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html#changing-the-cmake-code
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-packed-bitfield-compat -Wunused-parameter)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flexray_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time)

# Build
set(use_SYSTEM_BOOST TRUE)
include_directories(include
SYSTEM {Boost_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME} src/ixxat_gw.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs flexray_msgs)

# Install binaries
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()