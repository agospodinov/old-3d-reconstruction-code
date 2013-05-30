project(Xu)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
enable_testing()

file(GLOB_RECURSE XU_HEADERS "${PROJECT_SOURCE_DIR}/include" "include/*.h")

add_subdirectory(include)
add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(test)

include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/vendor/include")

if(WIN32)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/win32")
elseif(UNIX)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/linux")
elseif(APPLE)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/macosx")
endif()

add_definitions("-std=c++11")
