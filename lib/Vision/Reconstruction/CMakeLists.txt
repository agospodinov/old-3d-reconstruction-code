set(RECONSTRUCTION_LIB_SOURCES
    AbstractFeatureMatcher.cpp		AbstractFeatureMatcher.h
    SURFGPUFeatureMatcher.cpp		SURFGPUFeatureMatcher.h
    GFTTFeatureMatcher.cpp          GFTTFeatureMatcher.h
    AbstractDenseMatcher.cpp		AbstractDenseMatcher.h
    DenseMatcher.cpp                DenseMatcher.h
    SceneReconstructor.cpp          SceneReconstructor.h
    PoseEstimator.cpp               PoseEstimator.h
    BundleAdjuster.cpp              BundleAdjuster.h
    CameraCalibrator.cpp            CameraCalibrator.h

    IImageMatcher.cpp               IImageMatcher.h
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

if(WIN32)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/win32")
elseif(UNIX)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/linux")
elseif(APPLE)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/macosx")
endif()

add_definitions(${PCL_DEFINITIONS} "-std=c++11")

add_library(Reconstruction SHARED ${RECONSTRUCTION_LIB_SOURCES})

target_link_libraries(Reconstruction Core VisionCore VisionInput five-point-nister sba lapack ${PCL_LIBRARIES})
