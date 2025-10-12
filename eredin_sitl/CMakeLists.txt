cmake_minimum_required(VERSION 3.8)
project(eredin_sitl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(control_node src/control.cpp)
ament_target_dependencies(control_node rclcpp std_msgs geometry_msgs nav_msgs actuator_msgs visualization_msgs tf2_ros) 

add_executable(odom_formations_node src/odom_formations.cpp)
ament_target_dependencies(odom_formations_node rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

add_executable(fields_viz_node src/fields_viz.cpp)
ament_target_dependencies(fields_viz_node rclcpp std_msgs geometry_msgs visualization_msgs)

#ament_python_install_package(${PROJECT_NAME})

#install(PROGRAMS
#  eredin_sitl/script.py
#  DESTINATION lib/${PROJECT_NAME}
#)

install(TARGETS
  control_node
  odom_formations_node
  fields_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Formation.msg"
)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
