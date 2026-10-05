cmake_minimum_required(VERSION 3.16)
project(robot3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ROBOT3D_BUILD_VIZ   "Build the raylib viewer"  ON)
option(ROBOT3D_BUILD_TESTS "Build the GoogleTest suite" ON)

include(FetchContent)

# ---- robotics core: no rendering dependencies ----
add_library(robot3d_core
  src/Geometry.cpp src/Environment.cpp src/Lidar.cpp src/AStar.cpp
  src/Robot.cpp src/Controller.cpp src/Simulation.cpp)
target_include_directories(robot3d_core PUBLIC include)
if(MSVC)
  target_compile_options(robot3d_core PRIVATE /W4)
else()
  target_compile_options(robot3d_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- viewer ----
if(ROBOT3D_BUILD_VIZ)
  find_package(raylib 5.0 QUIET)
  if(NOT raylib_FOUND)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz)
    FetchContent_MakeAvailable(raylib)
  endif()
  add_executable(robot3d src/main.cpp)
  target_link_libraries(robot3d PRIVATE robot3d_core raylib)
endif()

# ---- tests ----
if(ROBOT3D_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz)
    FetchContent_MakeAvailable(googletest)
  endif()
  add_executable(robot3d_tests
    tests/test_geometry.cpp tests/test_lidar.cpp tests/test_astar.cpp tests/test_navigation.cpp)
  target_link_libraries(robot3d_tests PRIVATE robot3d_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(robot3d_tests)
endif()
