# Module 9: the C++ part of the rebuild. Builds two copies of the tests and the benchmark:
#   rebuild_tests / rebuild_bench                    -> your velocity_pi.hpp
#   rebuild_tests_solution / rebuild_bench_solution  -> solutions/ (to check your setup works)
cmake_minimum_required(VERSION 3.18)
project(orthosis_rebuild CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT MSVC)
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

include(FetchContent)
find_package(GTest QUIET)                          # apt install libgtest-dev
if(NOT TARGET GTest::gtest_main)                   # otherwise download it
  FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# The real firmware's header, if this folder sits in the full repo: the tests compare your port with it,
# and the benchmark times it.
set(FIRMWARE_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/emg_orthosis/orthosis_core.h)

foreach(variant mine solution)
  if(variant STREQUAL "mine")
    set(src_dir ${CMAKE_CURRENT_SOURCE_DIR})
    set(suffix "")
  else()
    set(src_dir ${CMAKE_CURRENT_SOURCE_DIR}/solutions)
    set(suffix "_solution")
  endif()
  add_executable(rebuild_tests${suffix} tests/test_velocity_pi.cpp)
  target_include_directories(rebuild_tests${suffix} PRIVATE ${src_dir})
  target_link_libraries(rebuild_tests${suffix} PRIVATE GTest::gtest_main)
  target_compile_definitions(rebuild_tests${suffix} PRIVATE FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/data/pi_fixture.txt")
  add_executable(rebuild_bench${suffix} bench/bench_velocity_pi.cpp)
  target_include_directories(rebuild_bench${suffix} PRIVATE ${src_dir})
  if(EXISTS ${FIRMWARE_CORE})
    target_compile_definitions(rebuild_tests${suffix} PRIVATE FIRMWARE_CORE="${FIRMWARE_CORE}")
    target_compile_definitions(rebuild_bench${suffix} PRIVATE FIRMWARE_CORE="${FIRMWARE_CORE}")
  endif()
endforeach()
