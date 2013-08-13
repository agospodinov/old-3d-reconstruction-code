#include "Antiderivative.h"

#include <cmath>
#include <limits>

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            Antiderivative::Antiderivative(const std::shared_ptr<IFunction> &function)
                : Antiderivative(function, cbrt(std::numeric_limits<double>::epsilon()))
            {
            }

            Antiderivative::Antiderivative(const std::shared_ptr<IFunction> &function, double stepSize)
                : function(function),
                  stepSize(stepSize)
            {
            }

            double Antiderivative::Evaluate(double x) const
            {

            }
        }
    }
}
