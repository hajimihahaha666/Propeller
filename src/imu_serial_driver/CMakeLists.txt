cmake_minimum_required(VERSION 3.8)
project(imu_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

add_executable(imu_serial_node
  src/imu_serial_node.c
  src/imu_filter.c
  src/imu_position.c
)

ament_target_dependencies(imu_serial_node
  rcl
  rclc
  sensor_msgs
  std_msgs
  std_srvs
  rosidl_runtime_c
)

target_link_libraries(imu_serial_node m)

install(TARGETS
  imu_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
