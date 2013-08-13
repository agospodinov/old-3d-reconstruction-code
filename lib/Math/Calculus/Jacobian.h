#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "Math/Calculus/IFunction.h"
#include "Math/Calculus/PartialDerivative.h"
#include "Math/LinearAlgebra/Matrix.h"
#include "Math/LinearAlgebra/Vector.h"

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            template <int InputSize, int OutputSize>
            class Jacobian : IFunction<Math::LinearAlgebra::Vector<InputSize>, Math::LinearAlgebra::Matrix<OutputSize, InputSize> >
            {
                public:
                    Jacobian(const MultivariateFunction<InputSize, OutputSize> &function)
                        : function(&function)
                    {

                    }

                    virtual ~Jacobian()
                    {
                    }

                    virtual Math::LinearAlgebra::Matrix<OutputSize, InputSize> Evaluate(Math::LinearAlgebra::Vector<InputSize> input) const
                    {
                        Math::LinearAlgebra::Matrix<OutputSize, InputSize> jacobian;

                        for (int i = 0; i < InputSize; i++)
                        {
                            PartialDerivative<InputSize, OutputSize> pd(*function);
                            jacobian.col(i) = pd.Evaluate(input, i);
                        }

                        return jacobian;
                    }

                private:
                    const MultivariateFunction<InputSize, OutputSize> * const function;

            };
        }
    }
}

#endif // JACOBIAN_H
