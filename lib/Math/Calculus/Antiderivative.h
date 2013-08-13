#ifndef ANTIDERIVATIVE_H
#define ANTIDERIVATIVE_H

#include <memory>

#include "Math/Calculus/IFunction.h"

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            class Antiderivative : public IFunction
            {
                public:
                    explicit Antiderivative(const std::shared_ptr<IFunction> &function);
                    Antiderivative(const std::shared_ptr<IFunction> &function, double stepSize);
                    
                    virtual double Evaluate(double x);
                private:
                    std::shared_ptr<IFunction> function;

                    const double stepSize;
            };
        }
    }
}

#endif // ANTIDERIVATIVE_H
