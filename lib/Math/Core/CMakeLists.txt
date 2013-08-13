set(MATH_LIB_CORE_SOURCES
    Number.cpp      Number.h
)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

add_library(MathCore OBJECT ${MATH_LIB_CORE_SOURCES})
set_target_properties(MathCore PROPERTIES COMPILE_FLAGS "-fPIC")
