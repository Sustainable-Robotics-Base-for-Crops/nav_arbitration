cmake_minimum_required(VERSION 3.8)
project(nav_arbitration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  std_msgs
  lifecycle_msgs
  nav_interfaces
  nav_lifecycle_manager
  simple_node
  yasmin)

find_package(ament_cmake REQUIRED)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

add_executable(${PROJECT_NAME}
  src/arbitration.cpp
  src/states/states_arbitration.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
