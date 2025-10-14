cmake_minimum_required(VERSION 3.10.0)

project(algorithm_utils)

add_compile_options(-g)

include_directories(${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(tests)
add_subdirectory(examples)
