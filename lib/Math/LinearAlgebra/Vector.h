#ifndef VECTOR_H
#define VECTOR_H

#include "Math/Core/Number.h"
#include "Math/LinearAlgebra/Matrix.h"

namespace Xu
{
    namespace Math
    {
        namespace LinearAlgebra
        {
            template <int Elements>
            using Vector = Matrix<Elements, 1>;
        }
    }
}

#endif // VECTOR_H
