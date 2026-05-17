cmake_minimum_required(VERSION 3.8)
project(auv_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- Service generation ---------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InjectFault.srv"
  DEPENDENCIES std_msgs
)

# ---- Controller library ---------------------------------------------------
add_library(auv_control_core SHARED
  src/ts_fuzzy.cpp
  src/thrust_allocator.cpp
  src/smc_robustifier.cpp
)
target_include_directories(auv_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(auv_control_core Eigen3::Eigen)

# ---- Main controller node -------------------------------------------------
add_executable(auv_controller_node src/auv_controller_node.cpp)
ament_target_dependencies(auv_controller_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs
  tf2
)
target_link_libraries(auv_controller_node auv_control_core Eigen3::Eigen)

# Link the service types (this generation scheme works for Humble).
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(auv_controller_node ${cpp_typesupport_target})

# ---- Reference generator --------------------------------------------------
add_executable(reference_generator_node src/reference_generator_node.cpp)
ament_target_dependencies(reference_generator_node
  rclcpp std_msgs
)

# ---- Standalone unit tests for the algorithmic core ----------------------
add_executable(test_allocator src/test_allocator.cpp)
target_link_libraries(test_allocator auv_control_core Eigen3::Eigen)
ament_target_dependencies(test_allocator rclcpp)

# ---- Install --------------------------------------------------------------
install(TARGETS
  auv_control_core
  auv_controller_node
  reference_generator_node
  test_allocator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Python helper scripts
install(PROGRAMS
  scripts/inject_fault_cli.py
  scripts/scenario_runner.py
  scripts/plot_from_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(auv_control_core)
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs
                          tf2 Eigen3 rosidl_default_runtime)

ament_package()
