#ifndef IFUNCTION_H
#define IFUNCTION_H

#include "Math/Core/Number.h"
#include "Math/LinearAlgebra/Vector.h"

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            template <typename InputType, typename OutputType>
            class IFunction
            {
                public:
                    IFunction()
                    {
                    }

                    virtual ~IFunction()
                    {
                    }

                    virtual OutputType Evaluate(InputType input) const = 0;
            };

            using UnivariateFunction = IFunction<Math::Core::Number, Math::Core::Number>;

            template <int InputSize, int OutputSize>
            using MultivariateFunction = IFunction<Math::LinearAlgebra::Vector<InputSize>, Math::LinearAlgebra::Vector<OutputSize> >;
        }
    }
}

#endif // IFUNCTION_H
