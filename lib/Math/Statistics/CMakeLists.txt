set(MATH_LIB_STATISTICS_SOURCES
    RANSAC.cpp      RANSAC.h
)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

add_library(Statistics OBJECT ${MATH_LIB_STATISTICS_SOURCES})
set_target_properties(Statistics PROPERTIES COMPILE_FLAGS "-fPIC")
