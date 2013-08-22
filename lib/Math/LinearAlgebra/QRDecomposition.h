#ifndef QRDECOMPOSITION_H
#define QRDECOMPOSITION_H

#include "Math/LinearAlgebra/Matrix.h"

#include <eigen3/Eigen/QR>

namespace Xu
{
    namespace Math
    {
        namespace LinearAlgebra
        {
            template <std::size_t Rows, std::size_t Columns>
            class QRDecomposition
            {
                public:
                    QRDecomposition(const Matrix<Rows, Columns> &matrix)
                        : qr(matrix.householderQr())
                    {

                    }

                    Matrix<Rows, Rows> GetQ() const
                    {
                        return qr.householderQ();
                    }

                    Matrix<Rows, Columns> GetR() const
                    {
//                        return qr.matrixQR().triangularView<Eigen::Upper>();
                    }

                private:
                    Eigen::HouseholderQR<Matrix<Rows, Columns> > qr;
            };
        }
    }
}

#endif // QRDECOMPOSITION_H
