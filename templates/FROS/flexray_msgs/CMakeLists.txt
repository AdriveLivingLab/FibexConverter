cmake_minimum_required(VERSION 3.0.2)
project(flexray_msgs)
add_compile_options(-std=c++17 -Wno-packed-bitfield-compat)														   
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

add_message_files(DIRECTORY msg
  FILES
    {% for element in msg_list -%}
        {{ element }}
    {% endfor %}
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)