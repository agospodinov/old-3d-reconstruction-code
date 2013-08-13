#ifndef MATRIX_H
#define MATRIX_H

#include <eigen3/Eigen/Dense>

#include "Math/Core/Number.h"

namespace Xu
{
    namespace Math
    {
        namespace LinearAlgebra
        {
            const int RuntimeSized = Eigen::Dynamic;

            template <int Rows, int Columns>
            using Matrix = Eigen::Matrix<double, Rows, Columns>;
        }
    }
}

#endif // MATRIX_H
