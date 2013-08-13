set(VISION_CORE_LIB_SOURCES
    SingleViewCamera.cpp	SingleViewCamera.h
    SingleViewImage.cpp		SingleViewImage.h
    PointOfView.cpp         PointOfView.h
    CameraParameters.cpp	CameraParameters.h
    Point.cpp               Point.h
    Projection.cpp          Projection.cpp
    PointCloud.cpp          PointCloud.h
    Feature.cpp             Feature.h
    FeatureSet.cpp      	FeatureSet.h
    Scene.cpp           	Scene.h

    ICamera.cpp         	ICamera.h
    IImage.cpp              IImage.h
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

include_directories("${PROJECT_SOURCE_DIR}/lib" ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIR} ${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions("-std=c++11")

add_library(VisionCore SHARED ${VISION_CORE_LIB_SOURCES})

target_link_libraries(VisionCore ${PCL_LIBRARIES})
