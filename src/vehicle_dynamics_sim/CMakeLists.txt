cmake_minimum_required(VERSION 3.8)
project(vehicle_dynamics_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wconversion -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set(dependencies
  rclcpp
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
  Boost
  fmt
)

#########
# build #
#########
# library
add_library(${PROJECT_NAME}
  src/conversions.cpp
  src/declare_and_get_parameter.cpp
  src/localization.cpp
  src/Pose2D.cpp
  src/sim.cpp
  src/urdf.cpp
  src/vehicles.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})

# executable
add_executable(sim_node
  src/sim_node.cpp
)
target_include_directories(sim_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(sim_node PRIVATE)
target_link_libraries(sim_node ${PROJECT_NAME} fmt::fmt)
ament_target_dependencies(sim_node ${dependencies})


###########
# install #
###########
# library
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# executable
install(TARGETS
  sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/publish_some_velocities
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME}
)

########
# test #
########
if(BUILD_TESTING)
  # C++ unit tests
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  set(test_dependencies
    ament_index_cpp
  )
  ament_add_gtest(test_all
    test/test_all.cpp
    test/test_Pose2D.cpp
    test/test_sim.cpp
  )
  target_link_libraries(test_all ${PROJECT_NAME})
  ament_target_dependencies(
      test_all ${dependencies} ${test_dependencies})

  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  # TODO put some here
  #add_ros_isolated_launch_test(test/test_in_simulation.py)
endif()

ament_package()
