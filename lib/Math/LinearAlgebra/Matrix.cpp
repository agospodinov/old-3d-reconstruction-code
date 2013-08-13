#include "Math/LinearAlgebra/Matrix.h"

#include <algorithm>
#include <stdexcept>

#include <opencv2/core/eigen.hpp>

namespace Xu
{
    namespace Math
    {
        namespace LinearAlgebra
        {
//            Matrix::Matrix()
//                : Matrix(0, 0)
//            {
//            }

//            Matrix::Matrix(int rows, int columns)
//                : VectorSpace({ rows, columns })
//            {
//            }

////            Matrix::Matrix(const cv::Mat &matrix)
////                : Matrix(matrix.rows, matrix.cols, matrix.channels())
////            {
////                cv::Mat tmp;
////                if (matrix.depth() == CV_8U)
////                {
////                    matrix.convertTo(tmp, CV_64FC(matrix.channels()), 1.0/255.0);
////                }
////                else if (matrix.depth() == CV_32F)
////                {
////                    matrix.convertTo(tmp, CV_64FC(matrix.channels()));
////                }
////                else if (matrix.depth() == CV_64F)
////                {
////                    tmp = matrix;
////                }
////                void *rawData = data->data();
////                uchar *ucharData = static_cast<uchar *>(rawData);
////                std::copy(tmp.data, tmp.data + (rows * columns * channels * sizeof(double)), ucharData);
////            }

////            Matrix::Matrix(const Eigen::MatrixXd &matrix)
////                : Matrix(matrix.rows(), matrix.cols())
////            {
////                Eigen::MatrixXd tmp = matrix.transpose();
////                std::copy(tmp.data(), tmp.data() + (rows * columns), data->data());
////            }

//            Matrix::Matrix(const Matrix &other)
//                : Matrix(other.GetRows(), other.GetColumns())
//            {
//            }

//            Matrix::Matrix(Matrix &&other)
//                : Matrix(other.GetRows(), other.GetColumns())
//            {
//            }


//            Matrix::~Matrix()
//            {
//            }

//            Matrix &Matrix::operator =(const Matrix &other)
//            {
//                Matrix tmp(other);
//                std::swap(data, tmp.data);
//                return *this;
//            }

//            Matrix &Matrix::operator =(Matrix &&other)
//            {
////                VectorSpace::operator =(other);
//                return *this;
//            }


//            Matrix Matrix::Identity(int rows, int columns)
//            {
//                Matrix matrix(rows, columns);
//                for (int i = 0; i < rows; i++)
//                {
//                    for (int j = 0; j < columns; j++)
//                    {
//                        matrix.Set(j, i, (i == j) ? 1.0 : 0.0);
//                    }
//                }
//                return matrix;
//            }

//            Matrix Matrix::Zeros(int rows, int columns)
//            {
//                Matrix matrix(rows, columns);
//                for (int i = 0; i < rows; i++)
//                {
//                    for (int j = 0; j < columns; j++)
//                    {
//                        matrix.Set(j, i, 0.0);
//                    }
//                }
//                return matrix;
//            }

//            Matrix Matrix::Ones(int rows, int columns)
//            {
//                Matrix matrix(rows, columns);
//                for (int i = 0; i < rows; i++)
//                {
//                    for (int j = 0; j < columns; j++)
//                    {
//                        matrix.Set(j, i, 1.0);
//                    }
//                }
//                return matrix;
//            }

//            Matrix Matrix::Transpose() const
//            {
//            }

//            Core::Number Matrix::Get(std::size_t x, std::size_t y) const
//            {
//                CheckRange(x, y);
//                return data->at(y * columns + x);
//            }

//            void Matrix::Set(std::size_t x, std::size_t y, const Core::Number &value)
//            {
//                CheckRange(x, y);
//                data->at(y * columns + x) = value;
//            }

//            size_t Matrix::GetRows() const
//            {
//                return VectorSpace::GetSizeOfDimension(0);
//            }

//            size_t Matrix::GetColumns() const
//            {
//                return VectorSpace::GetSizeOfDimension(1);
//            }

////            cv::Mat Matrix::ToCvMat() const
////            {
////                return cv::Mat(rows, columns, CV_64FC(channels), data->data()).clone();
////            }

////            Eigen::MatrixXd Matrix::ToEigenMatrix() const
////            {
////                if (channels != 1)
////                {
////                    throw std::length_error("Eigen does not support multi-channel matrices.");
////                }
////                return static_cast<Eigen::MatrixXd>(Eigen::Map<Eigen::MatrixXd>(data->data(), rows, columns)).transpose();
////            }

//            std::ostream &operator <<(std::ostream &out, const Matrix &matrix)
//            {
//                for (int i = 0; i < matrix.GetRows(); i++)
//                {
//                    for (int j = 0; j < matrix.GetColumns(); j++)
//                    {
//                        out << matrix.Get(j, i) << " ";
//                    }
//                    out << std::endl;
//                }
//                return out;
//            }

//            void Matrix::CheckRange(int x, int y) const
//            {
//                if (x < 0 || x > GetColumns() || y < 0 || y > GetRows())
//                {
//                    std::stringstream ss;
//                    ss << "Matrix element with coordinates (" << x << ", " << y << ") does not exist.";
//                    throw std::out_of_range(ss.str());
//                }
//            }
        }
    }
}
