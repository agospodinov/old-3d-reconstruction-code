#ifndef VECTOR_H
#define VECTOR_H

//#include <array>

//#include <boost/numeric/ublas/vector.hpp>

//#include "Math/LinearAlgebra/Field.h"

#include <eigen3/Eigen/Dense>

#include "Math/Core/Number.h"

namespace Xu
{
    namespace Math
    {
        namespace LinearAlgebra
        {
            typedef Eigen::Matrix<Math::Core::Number, Eigen::Dynamic, 1> Vector;

//            template <typename T, unsigned int N>
//            class Vector
//            {
//                public:
//                    Vector();

//                private:
//                    std::array<typename Field<T>::Scalar, N> values;

//            };
        }
    }
}

#endif // VECTOR_H
