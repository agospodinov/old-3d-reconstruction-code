#include "NumberOperationsTest.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "Math/Core/Number.h"

TEST_F(NumberOperationsTest, Assignment)
{
}


TEST_F(NumberOperationsTest, Addition)
{
    ASSERT_EQ(3, plusOne + plusTwo);
    ASSERT_EQ(1, plusTwo + minusOne);

    ASSERT_EQ(0, plusOne + minusOne);
    ASSERT_EQ(0, plusTwo + minusTwo);

    ASSERT_EQ(-3, minusTwo + minusOne);
    ASSERT_EQ(-1, minusTwo + plusOne);
}

TEST_F(NumberOperationsTest, Subtraction)
{
    ASSERT_EQ(1, plusTwo - plusOne);
    ASSERT_EQ(3, plusTwo - minusOne);

    ASSERT_EQ(0, plusOne - plusOne);
    ASSERT_EQ(0, minusOne - minusOne);

    ASSERT_EQ(-1, minusTwo - minusOne);
    ASSERT_EQ(-3, minusTwo - plusOne);
}

TEST_F(NumberOperationsTest, Multiplication)
{
    ASSERT_EQ(2, plusOne * plusTwo);
    ASSERT_EQ(-2, plusOne * minusTwo);
    ASSERT_EQ(0, plusOne * zero);

    ASSERT_EQ(2, minusOne * minusTwo);
    ASSERT_EQ(-2, minusOne * plusTwo);
    ASSERT_EQ(0, minusOne * zero);
}

TEST_F(NumberOperationsTest, Division)
{
    ASSERT_EQ(0.5, plusOne / plusTwo);
    ASSERT_EQ(2, plusTwo / plusOne);
    ASSERT_EQ(-0.5, plusOne / minusTwo);
    ASSERT_EQ(-2, plusTwo / minusOne);

    ASSERT_EQ(0.5, minusOne / minusTwo);
    ASSERT_EQ(2, minusTwo / minusOne);
    ASSERT_EQ(-0.5, minusOne / plusTwo);
    ASSERT_EQ(-2, minusTwo / plusOne);

//    ASSERT_THROW(plusOne / zero, std::invalid_argument);
//    ASSERT_THROW(minusOne / zero, std::invalid_argument);
}

TEST_F(NumberOperationsTest, Modulus)
{
}

TEST_F(NumberOperationsTest, EqualityComparison)
{

}

TEST_F(NumberOperationsTest, LessThanComparison)
{

}
