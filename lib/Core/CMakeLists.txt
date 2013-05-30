set(CORE_LIB_SOURCES
    Object.cpp
    ObjectData.cpp
    Fact.cpp
    Event.cpp
)

include_directories("${PROJECT_SOURCE_DIR}/include")
#link_directories(${LIBRARY_DIRS})
add_definitions("-std=c++11")

add_library(Core SHARED ${CORE_LIB_SOURCES} ${XU_HEADERS})

#target_link_libraries(Core ...)
