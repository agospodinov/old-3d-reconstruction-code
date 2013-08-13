set(VISION_INPUT_LIB_SOURCES
    CameraInputSource.cpp       CameraInputSource.h
    ImageListInputSource.cpp	ImageListInputSource.h

    IInputSource.cpp            IInputSource.h
)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

add_library(VisionInput SHARED ${VISION_INPUT_LIB_SOURCES})

