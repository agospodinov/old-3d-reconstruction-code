set(MATH_LIB_SOURCES
    Matrix.cpp
)

find_package(Eigen3 REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include" ${Eigen3_INCLUDE_DIR})
#link_directories()
add_definitions("-std=c++11")

add_library(Math SHARED ${MATH_LIB_SOURCES})

#target_link_libraries(Math ...)
