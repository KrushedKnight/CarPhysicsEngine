cmake_minimum_required(VERSION 3.10)
project(SimpleTrafficGame)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/eigen-3.4.0)

add_executable(SimpleTrafficGame main.cpp
        Car.cpp
        Car.h
        constants.h
        RigidBody.cpp
        RigidBody.h
        Wheel.cpp
        Wheel.h
        constants.cpp)
target_link_libraries(SimpleTrafficGame ${SDL2_LIBRARIES})
