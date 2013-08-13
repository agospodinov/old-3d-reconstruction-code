#include "DerivativeTest.h"

#include <gtest/gtest.h>

#include "Math/Calculus/IFunction.h"
#include "Math/Calculus/Derivative.h"

TEST(DerivativeTest, QuadraticFunction)
{
    class QuadraticFunction : public Xu::Math::Calculus::IFunction
    {
        public:
            virtual double Evaluate(double x)
            {
                return x * x;
            }
    };
    std::shared_ptr<Xu::Math::Calculus::IFunction> function = std::make_shared<QuadraticFunction>();

    Xu::Math::Calculus::Derivative derivative(function);

    class : public Xu::Math::Calculus::IFunction
    {
        public:
            virtual double Evaluate(double x)
            {
                return 2 * x;
            }
    } actualDerivative;

    ASSERT_NEAR(derivative.Evaluate(-10.0), actualDerivative.Evaluate(-10.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(-5.0), actualDerivative.Evaluate(-5.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(-2.0), actualDerivative.Evaluate(-2.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(-1.0), actualDerivative.Evaluate(-1.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(0.0), actualDerivative.Evaluate(0.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(1.0), actualDerivative.Evaluate(1.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(2.0), actualDerivative.Evaluate(2.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(5.0), actualDerivative.Evaluate(5.0), 0.00001);
    ASSERT_NEAR(derivative.Evaluate(10.0), actualDerivative.Evaluate(10.0), 0.00001);
}
