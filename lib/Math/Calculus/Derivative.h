#ifndef DERIVATIVE_H
#define DERIVATIVE_H

#include "Math/Calculus/IFunction.h"

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            class Derivative : public UnivariateFunction
            {
                public:
                    explicit Derivative(const UnivariateFunction &function);
                    Derivative(const UnivariateFunction &function, double stepSize);
                    virtual ~Derivative();

                    virtual Math::Core::Number Evaluate(Math::Core::Number x) const;

                private:
                    const UnivariateFunction * const function;

                    const Math::Core::Number stepSize;
            };
        }
    }
}

#endif // DERIVATIVE_H
