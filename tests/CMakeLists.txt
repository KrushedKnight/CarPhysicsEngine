# CMakeLists.txt for tests
cmake_minimum_required(VERSION 3.10)

# Enable testing
enable_testing()

# Download and configure Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Find SDL2 for linking
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/eigen-3.4.0)
include_directories(${SDL2_INCLUDE_DIRS})

# Create test executable
add_executable(
  RunAllTests
  RigidBodyTest.cpp
  WheelTest.cpp
  CarTest.cpp
  ${CMAKE_SOURCE_DIR}/src/core/RigidBody.cpp
  ${CMAKE_SOURCE_DIR}/src/vehicle/Wheel.cpp
  ${CMAKE_SOURCE_DIR}/src/vehicle/Car.cpp
  ${CMAKE_SOURCE_DIR}/src/vehicle/Engine.cpp
  ${CMAKE_SOURCE_DIR}/src/vehicle/Gearbox.cpp
  ${CMAKE_SOURCE_DIR}/src/control/TractionControl.cpp
  ${CMAKE_SOURCE_DIR}/src/control/AntiLockBrakes.cpp
  ${CMAKE_SOURCE_DIR}/src/rendering/Camera.cpp
  ${CMAKE_SOURCE_DIR}/src/config/constants.cpp
)

# Link Google Test and SDL2
target_link_libraries(
  RunAllTests
  gtest_main
  gtest
  pthread
  ${SDL2_LIBRARIES}
)

# Add tests to CTest
include(GoogleTest)
gtest_discover_tests(RunAllTests)