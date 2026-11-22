cmake_minimum_required(VERSION 3.8)
project(mpc_control)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED)

add_executable(mpc_control
  src/main.cpp
  src/mpc_controller.cpp
  src/reference_line.cpp
  src/mpc_osqp.cpp)

target_include_directories(mpc_control PRIVATE include)
target_compile_features(mpc_control PRIVATE cxx_std_17)
target_link_libraries(mpc_control Eigen3::Eigen osqp::osqp)
ament_target_dependencies(mpc_control
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2)

install(TARGETS mpc_control
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config data launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(mpc_preview_test
    test/mpc_preview_test.cpp
    src/mpc_controller.cpp
    src/mpc_osqp.cpp)
  target_include_directories(mpc_preview_test PRIVATE include)
  target_compile_features(mpc_preview_test PRIVATE cxx_std_17)
  target_link_libraries(mpc_preview_test Eigen3::Eigen osqp::osqp)
  add_test(NAME mpc_preview_test COMMAND mpc_preview_test)
endif()

ament_package()