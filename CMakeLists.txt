cmake_minimum_required(VERSION 3.20)
project(embree-tinyobj-example LANGUAGES C CXX)

# spdlog
if(NOT TARGET spdlog)
    find_package(spdlog REQUIRED)
endif()

# OpenMP
find_package(OpenMP)
if(NOT OpenMP_CXX_FOUND)
  message(WARNING "Could not find OpenMP")
endif()

# Embree3
find_package(embree 3.0 REQUIRED)
if (${embree_FOUND})
  message(STATUS "Found Embree")
else()
  message(FATAL_ERROR "Could not find Embree")
endif()

# externals
add_subdirectory("externals")

add_library(lib INTERFACE)
target_include_directories(lib INTERFACE "include")
target_compile_features(lib INTERFACE cxx_std_17)
set_target_properties(lib PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(lib INTERFACE spdlog::spdlog)
target_link_libraries(lib INTERFACE OpenMP::OpenMP_CXX)
target_link_libraries(lib INTERFACE embree)
target_link_libraries(lib INTERFACE tinyobjloader)

# examples
add_subdirectory("examples")

# tests
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()