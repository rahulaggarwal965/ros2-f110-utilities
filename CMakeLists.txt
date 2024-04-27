cmake_minimum_required(VERSION 3.5)
project(utilities)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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
  std_msgs
  visualization_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

add_library(${PROJECT_NAME} INTERFACE)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs
)

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_export_targets("export_${PROJECT_NAME}")
ament_export_dependencies(
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
