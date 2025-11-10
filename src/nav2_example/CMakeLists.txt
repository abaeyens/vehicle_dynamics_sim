cmake_minimum_required(VERSION 3.8)
project(nav2_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wconversion -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


###########
# install #
###########
install(
  PROGRAMS
    #scripts/publish_some_velocities
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
