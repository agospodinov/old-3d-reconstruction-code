#ifndef DERIVATIVE_H
#define DERIVATIVE_H

#include <memory>

#include "Math/Calculus/IFunction.h"

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            class Derivative : public IFunction
            {
                public:
                    explicit Derivative(const std::shared_ptr<IFunction> &function);
                    Derivative(const std::shared_ptr<IFunction> &function, double stepSize);
                    virtual ~Derivative();

                    virtual double Evaluate(double x);

                private:
                    std::shared_ptr<IFunction> function;

                    const double stepSize;
            };
        }
    }
}

#endif // DERIVATIVE_H
