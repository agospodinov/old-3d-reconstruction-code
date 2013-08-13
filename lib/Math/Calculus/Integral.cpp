#include "Integral.h"

#include <limits>
#include <cmath>

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            Integral::Integral(const UnivariateFunction &function)
                : Integral(function, cbrt(std::numeric_limits<double>::epsilon()))
            {
            }

            Integral::Integral(const UnivariateFunction &function, double stepSize)
                : function(&function),
                  stepSize(stepSize),
                  definite(false)
            {
            }

            Integral::Integral(const UnivariateFunction &function, double lowerLimit, double upperLimit)
                : Integral(function, lowerLimit, upperLimit, cbrt(std::numeric_limits<double>::epsilon()))
            {
            }

            Integral::Integral(const UnivariateFunction &function, double lowerLimit, double upperLimit, double stepSize)
                : function(&function),
                  stepSize(stepSize),
                  lowerLimit(lowerLimit),
                  upperLimit(upperLimit),
                  definite(true)
            {
            }

            Integral::~Integral()
            {
            }

            double Integral::Evaluate(double x) const
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
