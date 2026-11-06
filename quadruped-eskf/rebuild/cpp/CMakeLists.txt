# Module 9: the C++ part of the rebuild. Builds two copies of the tests and the benchmark:
#   rebuild_tests / rebuild_bench                    -> your predict.hpp and fd.hpp
#   rebuild_tests_solution / rebuild_bench_solution  -> solutions/ (to check your setup works)
cmake_minimum_required(VERSION 3.18)
project(eskf_rebuild CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT MSVC)
  add_compile_options(-Wall -Wextra)
endif()

include(FetchContent)
find_package(Eigen3 3.3 QUIET NO_MODULE)          # apt install libeigen3-dev
if(NOT TARGET Eigen3::Eigen)                       # otherwise download the headers
  FetchContent_Declare(eigen URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz DOWNLOAD_EXTRACT_TIMESTAMP TRUE
                       SOURCE_SUBDIR do-not-build)
  FetchContent_MakeAvailable(eigen)
  add_library(eigen_headers INTERFACE)
  target_include_directories(eigen_headers INTERFACE ${eigen_SOURCE_DIR})
  add_library(Eigen3::Eigen ALIAS eigen_headers)
endif()
find_package(GTest QUIET)                          # apt install libgtest-dev
if(NOT TARGET GTest::gtest_main)
  FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

foreach(variant mine solution)
  if(variant STREQUAL "mine")
    set(src_dir ${CMAKE_CURRENT_SOURCE_DIR})
    set(suffix "")
  else()
    set(src_dir ${CMAKE_CURRENT_SOURCE_DIR}/solutions)
    set(suffix "_solution")
  endif()
  add_executable(rebuild_tests${suffix} tests/test_predict.cpp)
  target_include_directories(rebuild_tests${suffix} PRIVATE ${src_dir} include)
  target_link_libraries(rebuild_tests${suffix} PRIVATE Eigen3::Eigen GTest::gtest_main)
  target_compile_definitions(rebuild_tests${suffix} PRIVATE FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/data/predict_fixture.txt")
  add_executable(rebuild_bench${suffix} bench/bench_predict.cpp)
  target_include_directories(rebuild_bench${suffix} PRIVATE ${src_dir} include)
  target_link_libraries(rebuild_bench${suffix} PRIVATE Eigen3::Eigen)
endforeach()
