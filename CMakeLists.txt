cmake_minimum_required(VERSION 3.5)
project(utilities)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

include_directories(include)

# add_libary(${PROJECT_NAME} SHARED
    # src/utilities.cpp
# )
# ament_target_dependencies(utilities
  # rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
# )

# install(TARGETS
#     utilities
#     DESTINATION lib/${PROJECT_NAME}
# )

ament_python_install_package(${PROJECT_NAME})

ament_package()
