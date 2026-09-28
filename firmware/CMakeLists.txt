# Host build of the orthosis control core: unit tests, closed-loop tests against
# the motor/limb model, a demo that writes traces for the README figures, and a
# syntax check of the sketch itself against stub Arduino headers.
cmake_minimum_required(VERSION 3.16)
project(orthosis_firmware_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  add_library(GTest::gtest_main ALIAS gtest_main)
endif()

enable_testing()
add_executable(firmware_tests sim/test_core.cpp sim/test_closed_loop.cpp sim/test_latency.cpp)
target_link_libraries(firmware_tests GTest::gtest_main)
target_compile_definitions(firmware_tests PRIVATE ORTH_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/sim/data")
include(GoogleTest)
gtest_discover_tests(firmware_tests)

add_executable(sim_demo sim/demo.cpp)

# Compiles emg_orthosis.ino as C++ against sim/arduino_stub. Catches C++ errors
# on a laptop; the real ESP32 compile runs in CI (.github/workflows/firmware.yml).
add_executable(sketch_syntax_check sim/sketch_check.cpp)
target_include_directories(sketch_syntax_check PRIVATE sim/arduino_stub emg_orthosis)

add_executable(latency_sweep sim/latency_sweep.cpp)
