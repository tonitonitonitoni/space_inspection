cmake_minimum_required(VERSION 3.8)
project(base_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(base_description REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)

install(DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME})


ament_package()
