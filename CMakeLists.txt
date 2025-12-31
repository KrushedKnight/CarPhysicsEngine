cmake_minimum_required(VERSION 3.10)
project(SimpleTrafficGame)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES
    main.cpp
    src/core/RigidBody.cpp
    src/vehicle/Wheel.cpp
    src/vehicle/Car.cpp
    src/vehicle/Engine.cpp
    src/vehicle/Gearbox.cpp
    src/control/TractionControl.cpp
    src/control/AntiLockBrakes.cpp
    src/ui/GUI.cpp
    src/ui/Graph.cpp
    src/ui/FreeBodyDiagram.cpp
    src/ui/Dial.cpp
    src/config/constants.cpp
    src/rendering/Camera.cpp
    src/rendering/Ground.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/eigen-3.4.0)
include_directories(${CMAKE_SOURCE_DIR}/include)

if(EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")

    add_executable(SimpleTrafficGame ${SOURCES})

    set_target_properties(SimpleTrafficGame PROPERTIES
        LINK_FLAGS "-s USE_SDL=2 -s USE_SDL_TTF=2 -s WASM=1 -s ALLOW_MEMORY_GROWTH=1 --preload-file ${CMAKE_SOURCE_DIR}/assets@/assets --shell-file ${CMAKE_SOURCE_DIR}/shell.html -O2"
    )
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SDL2 REQUIRED sdl2)
    pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

    include_directories(${SDL2_INCLUDE_DIRS})
    include_directories(${SDL2_TTF_INCLUDE_DIRS})

    link_directories(${SDL2_LIBRARY_DIRS})
    link_directories(${SDL2_TTF_LIBRARY_DIRS})

    add_executable(SimpleTrafficGame ${SOURCES})
    target_link_libraries(SimpleTrafficGame ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})

    add_subdirectory(tests)
endif()
