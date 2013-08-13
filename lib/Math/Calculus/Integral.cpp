#include "Integral.h"

#include <limits>
#include <cmath>

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            Integral::Integral(const std::shared_ptr<IFunction> &function)
                : Integral(function, cbrt(std::numeric_limits<double>::epsilon()))
            {
            }

            Integral::Integral(const std::shared_ptr<IFunction> &function, double stepSize)
                : function(function),
                  stepSize(stepSize),
                  definite(false)
            {
            }

            Integral::Integral(const std::shared_ptr<IFunction> &function, double lowerLimit, double upperLimit)
                : Integral(function, lowerLimit, upperLimit, cbrt(std::numeric_limits<double>::epsilon()))
            {
            }

            Integral::Integral(const std::shared_ptr<IFunction> &function, double lowerLimit, double upperLimit, double stepSize)
                : function(function),
                  stepSize(stepSize),
                  lowerLimit(lowerLimit),
                  upperLimit(upperLimit),
                  definite(true)
            {
            }

            Integral::~Integral()
            {
            }

            double Integral::Evaluate(double x)
            {
                if (definite)
                {
                    
                }
                else
                {
                    
                }
            }
        }
    }
}
