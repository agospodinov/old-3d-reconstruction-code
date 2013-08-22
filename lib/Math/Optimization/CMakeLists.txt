set(MATH_LIB_OPTIMIZATION_SOURCES
    LevenbergMarquardt.cpp  LevenbergMarquardt.h
)

find_package(Eigen3 REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/lib" ${Eigen3_INCLUDE_DIR})
#link_directories()
add_definitions("-std=c++11")

add_library(Optimization OBJECT ${MATH_LIB_OPTIMIZATION_SOURCES})
set_target_properties(Optimization PROPERTIES COMPILE_FLAGS "-fPIC")
