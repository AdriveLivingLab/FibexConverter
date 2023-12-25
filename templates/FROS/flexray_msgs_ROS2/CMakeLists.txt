cmake_minimum_required(VERSION 3.5)
project(flexray_msgs)

# set default C++ compiler options according to the recommendation by ROS: https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html#changing-the-cmake-code
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-packed-bitfield-compat)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
{% for element in msg_list -%}
"msg/{{element}}.msg"
{% endfor %}
DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()