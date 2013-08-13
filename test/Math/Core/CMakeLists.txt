#set(MATH_CORE_TESTS_SOURCES
#    DerivativeTest.cpp	DerivativeTest.h
#    IntegralTest.cpp	IntegralTest.h
#)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

add_executable(NumberOperationsTest ${PROJECT_SOURCE_DIR}/test/RunTests.cpp NumberOperationsTest.cpp NumberOperationsTest.h)
add_test(NumberOperationsTest NumberOperationsTest)
target_link_libraries(NumberOperationsTest Math gtest)

#add_library(CalculusTest STATIC ${MATH_CALCULUS_TESTS_SOURCES})
