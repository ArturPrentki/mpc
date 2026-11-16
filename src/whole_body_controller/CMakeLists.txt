cmake_minimum_required(VERSION 3.8)
project(whole_body_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  osqp_eigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  GIT_TAG        v0.11.0
  SYSTEM
)
set(BUILD_TESTING_SAVED ${BUILD_TESTING})
set(BUILD_TESTING OFF)
FetchContent_MakeAvailable(osqp_eigen)
set(BUILD_TESTING ${BUILD_TESTING_SAVED})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(wbc_control_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

# ---------------------------------------------------------------------------
# wbc_core: ROS-independent controller (unit tested, reusable on any robot)
# ---------------------------------------------------------------------------
add_library(wbc_core STATIC
  src/robot_model.cpp
  src/dynamic_system.cpp
  src/wbic.cpp
  src/trajectory_interpolator.cpp
  src/safety_supervisor.cpp
  src/wbc_core.cpp
)
set_target_properties(wbc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(wbc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(wbc_core PUBLIC cxx_std_17)
target_link_libraries(wbc_core PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  OsqpEigen::OsqpEigen
)

# ---------------------------------------------------------------------------
# ros2_control plugin
# ---------------------------------------------------------------------------
generate_parameter_library(wbc_parameters config/whole_body_params.yaml)

add_library(whole_body_controller SHARED src/whole_body_controller.cpp)
target_link_libraries(whole_body_controller
  wbc_core
  wbc_parameters
)
ament_target_dependencies(whole_body_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
  wbc_control_msgs
)
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(TARGETS whole_body_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wbc_core test/test_wbc_core.cpp TIMEOUT 300)
  target_compile_definitions(test_wbc_core PRIVATE
    WBC_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_link_libraries(test_wbc_core wbc_core)
endif()

ament_export_dependencies(rclcpp rclcpp_lifecycle controller_interface
  hardware_interface pluginlib)
ament_package()
