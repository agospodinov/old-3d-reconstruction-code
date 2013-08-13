#ifndef NUMBEROPERATIONSTEST_H
#define NUMBEROPERATIONSTEST_H

#include <gtest/gtest.h>

#include "Math/Core/Number.h"

class NumberOperationsTest : public ::testing::Test
{
    protected:
        virtual void SetUp()
        {
            plusOne = 1;
            minusOne = -1;
            zero = 0;
            plusTwo = 2;
            minusTwo = -2;
        }

        virtual void TearDown()
        {

        }

        Xu::Math::Core::Number plusOne;
        Xu::Math::Core::Number minusOne;
        Xu::Math::Core::Number zero;
        Xu::Math::Core::Number plusTwo;
        Xu::Math::Core::Number minusTwo;

};

#endif // NUMBEROPERATIONSTEST_H
