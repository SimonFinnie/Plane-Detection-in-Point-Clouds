cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

include_directories(~/Documents/Honours/Cosc450/Assignment2/Eigen/)

project(planeFinder)
add_executable(planeFinder
planeFinder.cpp
SimplePly.h
SimplePly.cpp
rply.h
rply.c
)




