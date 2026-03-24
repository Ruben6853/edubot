cmake_minimum_required(VERSION 3.8)
project(assignment)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories($<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)

add_executable(jacobian
        jacobian.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../ros_ws/src/controllers_new/src/kinematics.cpp
)

target_include_directories(jacobian PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../ros_ws/src/controllers_new/include/controllers_new)