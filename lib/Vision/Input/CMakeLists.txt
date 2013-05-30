set(VISION_INPUT_LIB_SOURCES
    CameraInputSource.cpp
    ImageListInputSource.cpp

    IInputSource.cpp
)

include_directories("${PROJECT_SOURCE_DIR}/include")
#link_directories()
add_definitions("-std=c++11")

add_library(VisionInput SHARED ${VISION_INPUT_LIB_SOURCES})

