project(Xu)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
enable_testing()

add_definitions("-std=c++11")

if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
endif()

add_subdirectory(lib)
add_subdirectory(src)
add_subdirectory(test)

include_directories("${PROJECT_SOURCE_DIR}/lib" "${PROJECT_SOURCE_DIR}/vendor/include")

if(WIN32)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/win32")
elseif(UNIX)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/linux")
elseif(APPLE)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/macosx")
endif()

