cmake_minimum_required(VERSION 2.6)
project(mobilerobot)

add_executable(mobilerobot main.cpp)

install(TARGETS mobilerobot RUNTIME DESTINATION bin)
