find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_subdirectory(Core)
add_subdirectory(Framework)
add_subdirectory(Math)
add_subdirectory(Language)
add_subdirectory(Vision)
add_subdirectory(Audio)
