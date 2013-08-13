set(MATH_LIB_LINEAR_ALGEBRA_SOURCES
    Field.cpp           Field.h
    Vector.cpp          Vector.h
    VectorSpace.cpp     VectorSpace.h
    Matrix.cpp          Matrix.h
)

find_package(Eigen3 REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/lib" ${Eigen3_INCLUDE_DIR})
#link_directories()
add_definitions("-std=c++11")

add_library(LinearAlgebra OBJECT ${MATH_LIB_LINEAR_ALGEBRA_SOURCES})
set_target_properties(LinearAlgebra PROPERTIES COMPILE_FLAGS "-fPIC")
