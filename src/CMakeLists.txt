set(GUI_SOURCES
    main.cpp
    MainWindow.cpp
    CVImageWidget.cpp
)

set(GUI_HEADERS
    MainWindow.h
    CVImageWidget.h
)
set(GUI_FORMS MainWindow.ui)
#set(GUI_RESOURCES images.qrc)

find_package(Qt4 REQUIRED)
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

qt4_wrap_cpp(GUI_HEADERS_MOC ${GUI_HEADERS})
qt4_wrap_ui(GUI_FORMS_HEADERS ${GUI_FORMS})
#qt4_add_resources(GUI_RESOURCES_RCC ${GUI_RESOURCES})

include(${QT_USE_FILE})

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/include" ${QT_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} "${PROJECT_SOURCE_DIR}/vendor/include")
link_directories(${PCL_LIBRARY_DIRS})

if(WIN32)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/win32")
elseif(UNIX)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/linux")
elseif(APPLE)
    link_directories("${PROJECT_SOURCE_DIR}/vendor/lib/macosx")
endif()

add_definitions(${PCL_DEFINITIONS} ${QT_DEFINITIONS} "-std=c++11")

add_executable(${PROJECT_NAME}
    ${GUI_SOURCES} 
    ${GUI_HEADERS_MOC} 
    ${GUI_FORMS_HEADERS}
#    ${GUI_RESOURCES_RCC}
)

target_link_libraries(${PROJECT_NAME} Core VisionCore VisionInput Reconstruction Recognition Math ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})
