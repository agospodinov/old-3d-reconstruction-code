include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

add_subdirectory(Core)
add_subdirectory(LinearAlgebra)
add_subdirectory(Calculus)
add_subdirectory(Statistics)

add_library(Math SHARED $<TARGET_OBJECTS:MathCore> $<TARGET_OBJECTS:LinearAlgebra> $<TARGET_OBJECTS:Calculus> $<TARGET_OBJECTS:Statistics>)

#target_link_libraries(Math ...)
