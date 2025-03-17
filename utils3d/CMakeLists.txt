cmake_minimum_required(VERSION 3.24)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(utils3d)

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)
