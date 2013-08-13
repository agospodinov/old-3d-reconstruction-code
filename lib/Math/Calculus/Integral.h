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
                    explicit Integral(const std::shared_ptr<IFunction> &function);
                    Integral(const std::shared_ptr<IFunction> &function, double stepSize);
                    Integral(const std::shared_ptr<IFunction> &function, double lowerLimit, double upperLimit);
                    Integral(const std::shared_ptr<IFunction> &function, double lowerLimit, double upperLimit, double stepSize);
                    virtual ~Integral();

                    virtual double Evaluate(double x);

                private:
                    std::shared_ptr<IFunction> function;

                    const double stepSize;

                    bool definite;
                    double lowerLimit;
                    double upperLimit;
            };
        }
    }
}

#endif // INTEGRAL_H
