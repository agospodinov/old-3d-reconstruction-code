set(RECOGNITION_LIB_SOURCES
    StaticObjectDetector.cpp		StaticObjectDetector.h
    MovingObjectDetector.cpp		MovingObjectDetector.h
    ShapeBasedObjectDetector.cpp	ShapeBasedObjectDetector.h
    CompositeObjectDetector.cpp		CompositeObjectDetector.h

    IObjectDetector.cpp         	IObjectDetector.h
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories("${PROJECT_SOURCE_DIR}/lib" ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} "${PROJECT_SOURCE_DIR}/vendor/include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} "-std=c++11")

add_library(Recognition SHARED ${RECOGNITION_LIB_SOURCES})

target_link_libraries(Recognition Core VisionCore ${PCL_LIBRARIES})
