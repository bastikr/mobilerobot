cmake_minimum_required(VERSION 2.9)
project(mobilerobot)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-Wall -Wextra -Wfloat-equal -Wundef -Wcast-align -Wwrite-strings -Wlogical-op -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual")

include_directories(include)

add_library(mobilerobot SHARED src/filters.cc src/sensors.cc src/world.cc )

# Find the QtWidgets library
find_package(Qt5Widgets)
find_package(Qt5Charts)

add_executable(kalman examples/kalmanfilter.cc mobilerobot)

# Use the Widgets module from Qt 5.
target_link_libraries(kalman Qt5::Widgets Qt5::Charts mobilerobot)
