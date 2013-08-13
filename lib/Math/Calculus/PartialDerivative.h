#ifndef PARTIALDERIVATIVE_H
#define PARTIALDERIVATIVE_H

#include <cmath>
#include <limits>

#include "Math/Calculus/IFunction.h"

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            template <int InputSize, int OutputSize>
            class PartialDerivative : public MultivariateFunction<InputSize, OutputSize>
            {
                public:
                    PartialDerivative(const MultivariateFunction<InputSize, OutputSize> &function)
                        : function(&function),
                          stepSize(sqrt(std::numeric_limits<double>::epsilon()))
                    {

                    }

                    virtual ~PartialDerivative()
                    {
                    }

                    virtual Math::LinearAlgebra::Vector<OutputSize> Evaluate(Math::LinearAlgebra::Vector<InputSize> input, int variableIndex) const
                    {
                        if (variableIndex >= InputSize || variableIndex < 0)
                        {
                            throw std::out_of_range("Variable index out of range.");
                        }

                        Math::LinearAlgebra::Vector<InputSize> inputMinusDelta = input;
                        inputMinusDelta(variableIndex) -= stepSize;

                        Math::LinearAlgebra::Vector<InputSize> inputPlusDelta = input;
                        inputPlusDelta(variableIndex) += stepSize;

                        return (function->Evaluate(inputPlusDelta) - function->Evaluate(inputMinusDelta)) / (2 * stepSize);
                    }

                private:
                    const MultivariateFunction<InputSize, OutputSize> * const function;
                    const double stepSize;
            };
        }
    }
}

#endif // PARTIALDERIVATIVE_H
