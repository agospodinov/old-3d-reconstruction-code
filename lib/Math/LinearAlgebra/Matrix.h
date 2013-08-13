#ifndef MATRIX_H
#define MATRIX_H

//#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>

//#include <memory>

#include "Math/Core/Number.h"
//#include "VectorSpace.h"

namespace Xu
{
    namespace Math
    {
        namespace LinearAlgebra
        {
            template <int Rows, int Columns>
            using Matrix = Eigen::Matrix<double, Rows, Columns>;

//            using Matrix = Eigen::Matrix<Math::Core::Number, Eigen::Dynamic, Eigen::Dynamic>;

//            class Matrix : VectorSpace<2>
//            {
//                public:
//                    Matrix();
//                    Matrix(int rows, int columns);

////                    explicit Matrix(const cv::Mat &matrix);
////                    explicit Matrix(const Eigen::MatrixXd &matrix);

//                    Matrix(const Matrix &other);
//                    Matrix(Matrix &&other);

//                    ~Matrix();

//                    Matrix &operator= (const Matrix &other);
//                    Matrix &operator= (Matrix &&other);

//                    static Matrix Identity(int rows, int columns);
//                    static Matrix Zeros(int rows, int columns);
//                    static Matrix Ones(int rows, int columns);

//                    Matrix Transpose() const;

//                    Math::Core::Number Get(std::size_t x, std::size_t y) const;

//                    void Set(std::size_t x, std::size_t y, const Math::Core::Number &value);

//                    size_t GetRows() const;
//                    size_t GetColumns() const;

////                    cv::Mat ToCvMat() const;
////                    Eigen::MatrixXd ToEigenMatrix() const;

//                    void Release();

//                private:
//                    friend std::ostream &operator<<(std::ostream &os, const Matrix &matrix);

//                    void CheckRange(int x, int y) const;

//            };
        }
    }
}

#endif // MATRIX_H
