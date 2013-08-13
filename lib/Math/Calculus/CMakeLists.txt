set(MATH_LIB_CALCULUS_SOURCES
    IFunction.cpp           IFunction.h
    Derivative.cpp          Derivative.h
    Antiderivative.cpp      Antiderivative.h
    Integral.cpp            Integral.h
    PartialDerivative.cpp   PartialDerivative.h
    Jacobian.cpp            Jacobian.h
)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

add_library(Calculus OBJECT ${MATH_LIB_CALCULUS_SOURCES})
set_target_properties(Calculus PROPERTIES COMPILE_FLAGS "-fPIC")
