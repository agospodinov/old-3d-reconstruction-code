#set(MATH_TESTS_SOURCES
#    TestMain.cpp
#    Calculus/DerivativeTest.cpp	Calculus/DerivativeTest.h
#    Calculus/IntegralTest.cpp	Calculus/IntegralTest.h
#)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

add_subdirectory(Core)
#add_subdirectory(LinearAlgebra)
add_subdirectory(Calculus)

#add_executable(MathTest ${MATH_TESTS_SOURCES} ${MATH_CALCULUS_TESTS_SOURCES})
#add_test(MathLibTest MathTest)
#target_link_libraries(MathTest Math gtest)
