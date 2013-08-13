#ifndef INTEGRAL_H
#define INTEGRAL_H

#include <memory>

#include "Math/Calculus/IFunction.h"

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            class Integral
            {
                public:
                    explicit Integral(const UnivariateFunction &function);
                    Integral(const UnivariateFunction &function, double stepSize);
                    Integral(const UnivariateFunction &function, double lowerLimit, double upperLimit);
                    Integral(const UnivariateFunction &function, double lowerLimit, double upperLimit, double stepSize);
                    virtual ~Integral();

                    virtual double Evaluate(double x) const;

                private:
                    const UnivariateFunction * const function;

                    const double stepSize;

                    bool definite;
                    double lowerLimit;
                    double upperLimit;
            };
        }
    }
}

#endif // INTEGRAL_H
