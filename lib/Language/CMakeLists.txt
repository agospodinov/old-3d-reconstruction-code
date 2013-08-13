set(LANGUAGE_LIB_SOURCES
    Expression.cpp  Expression.h
    Context.cpp     Context.h
)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories(${LIBRARY_DIRS})
add_definitions("-std=c++11")

add_library(Language SHARED ${LANGUAGE_LIB_SOURCES})

#target_link_libraries(Core ...)
