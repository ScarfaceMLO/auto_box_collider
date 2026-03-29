cmake_minimum_required(VERSION 3.15)
project(autocol_core VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release build with optimizations by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(autocol_core SHARED
    autocol_api.cpp
    autocol_obb.cpp
    autocol_cluster.cpp
)

target_include_directories(autocol_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Optimization flags
if(MSVC)
    target_compile_options(autocol_core PRIVATE /O2 /W3)
else()
    target_compile_options(autocol_core PRIVATE -O3 -Wall -fvisibility=hidden)
endif()

# Output the DLL directly to the autocol folder (including MSVC per-config subdirs)
set_target_properties(autocol_core PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY                "${CMAKE_SOURCE_DIR}/../autocol"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/../autocol"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_SOURCE_DIR}/../autocol"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/../autocol"
    LIBRARY_OUTPUT_DIRECTORY                "${CMAKE_SOURCE_DIR}/../autocol"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/../autocol"
    PREFIX ""
)

install(TARGETS autocol_core DESTINATION ${CMAKE_SOURCE_DIR}/../autocol)
