cmake_minimum_required(VERSION 3.8)
project(rosey_traction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (the node only; the core library has no ROS dependencies)
set(NODE_DEPENDS
  control_msgs
  geometry_msgs
  rclcpp
  rosey_traction_msgs
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${NODE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# The estimator runs every cycle on the Pi; build optimized unless told otherwise.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

## CORE LIBRARY (plain C++17, no ROS dependencies)
add_library(
  rosey_traction_core
  SHARED
  src/terramechanics.cpp
  src/slip_estimator.cpp
  src/slip_table_lookup.cpp
  src/traction_limiter.cpp
  src/traction_core.cpp
)
target_compile_features(rosey_traction_core PUBLIC cxx_std_17)
target_include_directories(rosey_traction_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

## NODE
add_executable(traction_node src/traction_node.cpp)
target_link_libraries(traction_node rosey_traction_core)
ament_target_dependencies(traction_node ${NODE_DEPENDS})

## INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS rosey_traction_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS traction_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

## TESTS (ctest names start with software_ to match the CI filter on main)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_DATA_DIR "${PROJECT_SOURCE_DIR}/test/data")

  ament_add_gtest(software_terramechanics test/test_terramechanics.cpp)
  ament_add_gtest(software_slip_estimator test/test_slip_estimator.cpp)
  ament_add_gtest(software_slip_table test/test_slip_table.cpp)
  ament_add_gtest(software_traction_limiter test/test_traction_limiter.cpp)
  ament_add_gtest(software_traction_core test/test_traction_core.cpp)
  ament_add_gtest(software_traction_replay test/test_traction_replay.cpp)
  foreach(t software_terramechanics software_slip_estimator software_slip_table
      software_traction_limiter software_traction_core software_traction_replay)
    target_link_libraries(${t} rosey_traction_core)
    target_compile_definitions(${t} PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  endforeach()

  # Timing check, run by hand: build/rosey_traction/bench_estimator
  add_executable(bench_estimator test/bench_estimator.cpp)
  target_link_libraries(bench_estimator rosey_traction_core)

  # Slip-ramp replay trace for checking and tuning, run by hand: build/rosey_traction/replay_sim
  add_executable(replay_sim test/replay_sim.cpp)
  target_link_libraries(replay_sim rosey_traction_core)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
