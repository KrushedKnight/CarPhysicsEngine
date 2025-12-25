cmake_minimum_required(VERSION 3.10)
project(SimpleTrafficGame)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${SDL2_TTF_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/eigen-3.4.0)
include_directories(${CMAKE_SOURCE_DIR}/include)

link_directories(${SDL2_LIBRARY_DIRS})
link_directories(${SDL2_TTF_LIBRARY_DIRS})

add_executable(SimpleTrafficGame main.cpp
        src/core/RigidBody.cpp
        src/vehicle/Wheel.cpp
        src/vehicle/Car.cpp
        src/control/TractionControl.cpp
        src/control/AntiLockBrakes.cpp
        src/ui/GUI.cpp
        src/ui/Graph.cpp
        src/ui/FreeBodyDiagram.cpp
        src/ui/Dial.cpp
        src/config/constants.cpp
        src/rendering/Camera.cpp
        include/core/RigidBody.h
        include/vehicle/Wheel.h
        include/vehicle/Car.h
        include/control/TractionControl.h
        include/control/AntiLockBrakes.h
        include/ui/GUI.h
        include/ui/Graph.h
        include/ui/FreeBodyDiagram.h
        include/ui/Dial.h
        include/config/constants.h
        include/rendering/Camera.h
        include/vehicle/Engine.h
        src/vehicle/Engine.cpp
        include/vehicle/Gearbox.h
        src/vehicle/Gearbox.cpp
        include/config/EngineConstants.h)
target_link_libraries(SimpleTrafficGame ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})

# Add tests subdirectory
add_subdirectory(tests)
