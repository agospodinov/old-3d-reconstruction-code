#include "Derivative.h"

#include <limits>
#include <cmath>

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            Derivative::Derivative(const std::shared_ptr<IFunction> &function)
                : Derivative(function, cbrt(std::numeric_limits<double>::epsilon()))
            {
            }

            Derivative::Derivative(const std::shared_ptr<IFunction> &function, double stepSize)
                : function(function),
                  stepSize(stepSize)
            {
            }

            Derivative::~Derivative()
            {
            }

            double Derivative::Evaluate(double x)
            {
                return (function->Evaluate(x + stepSize) - function->Evaluate(x - stepSize)) / (2 * stepSize);
            }

        }
    }
}
