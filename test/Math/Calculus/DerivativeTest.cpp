#include "DerivativeTest.h"

#include <gtest/gtest.h>

#include "Math/Core/Number.h"
#include "Math/Calculus/IFunction.h"
#include "Math/Calculus/Derivative.h"

TEST(DerivativeTest, QuadraticFunction)
{
    class : public Xu::Math::Calculus::UnivariateFunction
    {
        public:
            virtual Xu::Math::Core::Number Evaluate(Xu::Math::Core::Number x) const
            {
                return x * x;
            }
    } quadraticFunction;
    Xu::Math::Calculus::Derivative derivative(quadraticFunction);

    class : public Xu::Math::Calculus::UnivariateFunction
    {
        public:
            virtual Xu::Math::Core::Number Evaluate(Xu::Math::Core::Number x) const
            {
                return 2 * x;
            }
    } actualDerivative;

    double threshold = 1e-10;
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(-10.0)), static_cast<double>(actualDerivative.Evaluate(-10.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(-5.0)), static_cast<double>(actualDerivative.Evaluate(-5.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(-2.0)), static_cast<double>(actualDerivative.Evaluate(-2.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(-1.0)), static_cast<double>(actualDerivative.Evaluate(-1.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(0.0)), static_cast<double>(actualDerivative.Evaluate(0.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(1.0)), static_cast<double>(actualDerivative.Evaluate(1.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(2.0)), static_cast<double>(actualDerivative.Evaluate(2.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(5.0)), static_cast<double>(actualDerivative.Evaluate(5.0)), threshold);
    ASSERT_NEAR(static_cast<double>(derivative.Evaluate(10.0)), static_cast<double>(actualDerivative.Evaluate(10.0)), threshold);
}
