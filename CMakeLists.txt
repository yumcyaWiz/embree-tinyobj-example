cmake_minimum_required(VERSION 3.20)
project(embree-tinyobj-example LANGUAGES C CXX)

# OpenMP
find_package(OpenMP)
if(NOT OpenMP_CXX_FOUND)
  message(WARNING "failed to find OpenMP")
endif()

# externals
add_subdirectory("externals")

add_library(lib INTERFACE)
target_include_directories(lib INTERFACE "include")
target_compile_features(lib INTERFACE cxx_std_17)
set_target_properties(lib PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(lib INTERFACE OpenMP::OpenMP_CXX)

# examples
add_subdirectory("examples")