#set(MATH_CALCULUS_TESTS_SOURCES
#    DerivativeTest.cpp	DerivativeTest.h
#    IntegralTest.cpp	IntegralTest.h
#)

include_directories("${PROJECT_SOURCE_DIR}/lib")
#link_directories()
add_definitions("-std=c++11")

#create_google_test(DerivativeTest "DerivativeTest.cpp DerivativeTest.h" "Math")
add_executable(DerivativeTest ${PROJECT_SOURCE_DIR}/test/RunTests.cpp DerivativeTest.cpp DerivativeTest.h)
add_test(DerivativeTest DerivativeTest)
target_link_libraries(DerivativeTest Math gtest)

add_executable(IntegralTest ${PROJECT_SOURCE_DIR}/test/RunTests.cpp IntegralTest.cpp IntegralTest.h)
add_test(IntegralTest IntegralTest)
target_link_libraries(IntegralTest Math gtest)
