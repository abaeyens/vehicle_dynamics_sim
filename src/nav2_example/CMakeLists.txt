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
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
