cmake_minimum_required(VERSION 2.9)
project(mobilerobot)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-Wall -Wextra -Wfloat-equal -Wundef -Wcast-align -Wwrite-strings -Wlogical-op -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual")

include_directories(include)

add_library(mobilerobot SHARED src/filters.cc src/sensors.cc src/world.cc src/worldmodel.cc src/sensormodel.cc)

# Find the QtWidgets library
find_package(Qt5Widgets)
find_package(Qt5Charts)

add_executable(filters examples/comparefilters.cc mobilerobot)
add_executable(particlefilter examples/particlefilter.cc mobilerobot)

# Use the Widgets module from Qt 5.
target_link_libraries(filters Qt5::Widgets Qt5::Charts mobilerobot)
target_link_libraries(particlefilter Qt5::Widgets Qt5::Charts mobilerobot)
