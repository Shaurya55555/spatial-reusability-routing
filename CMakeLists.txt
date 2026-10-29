cmake_minimum_required(VERSION 3.14)
project(spatial_reusability_routing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(srr src/network.cpp src/routing.cpp)
target_include_directories(srr PUBLIC include)
if(NOT MSVC)
  target_compile_options(srr PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(spatial_routing src/main.cpp)
target_link_libraries(spatial_routing PRIVATE srr)

enable_testing()
add_executable(srr_tests tests/test_main.cpp)
target_link_libraries(srr_tests PRIVATE srr)
target_compile_definitions(srr_tests PRIVATE TOPOLOGY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/topologies")
add_test(NAME srr_tests COMMAND srr_tests)
