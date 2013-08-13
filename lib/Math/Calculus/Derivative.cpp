#include "Derivative.h"

#include <limits>
#include <cmath>

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            Derivative::Derivative(const UnivariateFunction &function)
                : Derivative(function, sqrt(std::numeric_limits<double>::epsilon()))
            {
            }

            Derivative::Derivative(const UnivariateFunction &function, double stepSize)
                : function(&function),
                  stepSize(stepSize)
            {
            }

            Derivative::~Derivative()
            {
            }

            Core::Number Derivative::Evaluate(Core::Number x) const
            {
                return (function->Evaluate(x + stepSize) - function->Evaluate(x - stepSize)) / (2 * stepSize);
            }

        }
    }
}
