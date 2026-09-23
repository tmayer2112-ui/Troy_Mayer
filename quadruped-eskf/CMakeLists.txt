cmake_minimum_required(VERSION 3.16)
project(qeskf CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(qeskf cpp/src/eskf.cpp cpp/src/kinematics.cpp)
target_include_directories(qeskf PUBLIC cpp/include)
target_link_libraries(qeskf PUBLIC Eigen3::Eigen)

add_executable(qeskf_replay cpp/apps/replay.cpp)
target_link_libraries(qeskf_replay PRIVATE qeskf)

add_executable(qeskf_bench_loop cpp/apps/bench_loop.cpp)
target_link_libraries(qeskf_bench_loop PRIVATE qeskf)

option(QESKF_BUILD_TESTS "Build GoogleTest suite" ON)
if(QESKF_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()
  add_executable(qeskf_tests cpp/tests/test_jacobians.cpp cpp/tests/test_invariants.cpp cpp/tests/test_parity.cpp)
  target_link_libraries(qeskf_tests PRIVATE qeskf GTest::gtest_main)
  target_compile_definitions(qeskf_tests PRIVATE QESKF_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/cpp/tests/data")
  include(GoogleTest)
  gtest_discover_tests(qeskf_tests)
endif()
