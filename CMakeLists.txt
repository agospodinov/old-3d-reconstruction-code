project(Xu)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_subdirectory(src)

include_directories(${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/vendor/include")
link_directories("${PROJECT_SOURCE_DIR}/vendor/lib")
add_definitions("-std=c++11")
