set(CORE_LIB_SOURCES
    Object.cpp		Object.h
    ObjectData.cpp	ObjectData.h
    Fact.cpp		Fact.h
    Event.cpp		Event.h
)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories(${LIBRARY_DIRS})
add_definitions("-std=c++11")

add_library(Core SHARED ${CORE_LIB_SOURCES})

#target_link_libraries(Core ...)
