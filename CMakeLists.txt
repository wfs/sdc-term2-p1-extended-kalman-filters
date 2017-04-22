project(ExtendedKF)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++0x)

# BDD-Style Tests
#set(sources src/test_tools.cpp)
#set(sources src/test_ekf.cpp)

# SDC Term 2 Project 1 - Extended Kalman Filters
set(sources src/FusionEKF.cpp src/kalman_filter.cpp src/main.cpp src/tools.cpp)

add_executable(ExtendedKF ${sources})
