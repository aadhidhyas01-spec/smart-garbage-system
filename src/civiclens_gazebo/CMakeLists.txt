cmake_minimum_required(VERSION 3.10)
project(civiclens_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)

include_directories(include)

# 1. Autonomy Node
add_executable(civiclens_autonomy src/civiclens_autonomy.cpp)
target_link_libraries(civiclens_autonomy rclcpp::rclcpp std_msgs::std_msgs geometry_msgs::geometry_msgs)

# 2. Waste Manager Node
add_executable(waste_manager src/waste_manager.cpp)
target_link_libraries(waste_manager rclcpp::rclcpp std_msgs::std_msgs)

# 3. New Gazebo System Plugin
add_library(truck_gazebo_plugin SHARED src/truck_gazebo_plugin.cpp)
target_link_libraries(truck_gazebo_plugin
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  gz-math::gz-math
  gz-transport::gz-transport
  gz-msgs::gz-msgs
)

install(DIRECTORY launch worlds models config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  civiclens_autonomy
  waste_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/civiclens_ai_vision.py
  scripts/civiclens_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  truck_gazebo_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
