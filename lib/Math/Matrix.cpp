#include "Math/Matrix.h"

#include <algorithm>
#include <stdexcept>

#include <opencv2/core/eigen.hpp>

namespace Xu
{
    namespace Math
    {
        Matrix::Matrix()
            : rows(0),
              columns(0),
              channels(0)
        {
        }

        Matrix::Matrix(int rows, int cols, int channels)
            : rows(rows),
              columns(cols),
              channels(channels),
              data(std::shared_ptr<std::vector<double> >(new std::vector<double>(cols * rows * channels)))
        {
        }

        Matrix::Matrix(const cv::Mat &matrix)
            : Matrix(matrix.rows, matrix.cols, matrix.channels())
        {
            cv::Mat tmp;
            if (matrix.depth() == CV_8U)
            {
                matrix.convertTo(tmp, CV_64FC(matrix.channels()), 1.0/255.0);
            }
            else if (matrix.depth() == CV_32F)
            {
                matrix.convertTo(tmp, CV_64FC(matrix.channels()));
            }
            else if (matrix.depth() == CV_64F)
            {
                tmp = matrix;
            }
            void *rawData = data->data();
            uchar *ucharData = static_cast<uchar *>(rawData);
            std::copy(tmp.data, tmp.data + (rows * columns * channels * sizeof(double)), ucharData);
        }

        Matrix::Matrix(const Eigen::MatrixXd &matrix)
            : Matrix(matrix.rows(), matrix.cols(), 1)
        {
            Eigen::MatrixXd tmp = matrix.transpose();
            std::copy(tmp.data(), tmp.data() + (rows * columns), data->data());
        }

        Matrix::Matrix(const Matrix &other)
            : rows(other.rows),
              columns(other.columns),
              channels(other.channels),
              data(other.data)
        {
        }

        Matrix::Matrix(Matrix &&other)
            : Matrix(other.rows, other.columns, other.channels)
        {
            std::swap(data, other.data);
        }


        Matrix::~Matrix()
        {
        }

        Matrix &Matrix::operator =(const Matrix &other)
        {
            Matrix tmp(other);
            std::swap(data, tmp.data);
            return *this;
        }

        Matrix &Matrix::operator =(Matrix &&other)
        {
            rows = other.rows;
            columns = other.columns;
            channels = other.channels;
            std::swap(data, other.data);
            return *this;
        }


        Matrix Matrix::Identity(int rows, int columns)
        {
            Matrix matrix(rows, columns, 1);
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < columns; j++)
                {
                    matrix.Set(j, i, (i == j) ? 1.0 : 0.0);
                }
            }
            return matrix;
        }

        Matrix Matrix::Zeros(int rows, int columns)
        {
            Matrix matrix(rows, columns, 1);
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < columns; j++)
                {
                    matrix.Set(j, i, 0.0);
                }
            }
            return matrix;
        }

        Matrix Matrix::Ones(int rows, int columns)
        {
            Matrix matrix(rows, columns, 1);
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < columns; j++)
                {
                    matrix.Set(j, i, 1.0);
                }
            }
            return matrix;
        }

        Matrix Matrix::Transpose() const
        {
        }

        double Matrix::Get(int x, int y) const
        {
            CheckRange(x, y);
            return data->at(y * columns + x);
        }

        double Matrix::Get(int x, int y, int channel) const
        {
            CheckRange(x, y);
            return data->at(y * columns * channels + x * channels + channel);
        }

        void Matrix::Set(int x, int y, double value)
        {
            CheckRange(x, y);
            data->at(y * columns + x) = value;
        }

        void Matrix::Set(int x, int y, int channel, double value)
        {
            CheckRange(x, y);
            data->at(y * columns * channels + x * channels + channel) = value;
        }

        size_t Matrix::GetRows() const
        {
            return rows;
        }

        size_t Matrix::GetColumns() const
        {
            return columns;
        }

        size_t Matrix::GetChannels() const
        {
            return channels;
        }

        cv::Mat Matrix::ToCvMat() const
        {
            return cv::Mat(rows, columns, CV_64FC(channels), data->data()).clone();
        }

        Eigen::MatrixXd Matrix::ToEigenMatrix() const
        {
            if (channels != 1)
            {
                throw std::length_error("Eigen does not support multi-channel matrices.");
            }
            return static_cast<Eigen::MatrixXd>(Eigen::Map<Eigen::MatrixXd>(data->data(), rows, columns)).transpose();
        }

        void Matrix::Release()
        {
            data.reset();
        }

        std::ostream &operator <<(std::ostream &out, const Matrix &matrix)
        {
            for (int i = 0; i < matrix.rows; i++)
            {
                for (int j = 0; j < matrix.columns; j++)
                {
                    out << matrix.Get(j, i) << " ";
                }
                out << std::endl;
            }
            return out;
        }

        void Matrix::CheckRange(int x, int y) const
        {
            if (x < 0 || x > columns || y < 0 || y > rows)
            {
                std::stringstream ss;
                ss << "Matrix element with coordinates (" << x << ", " << y << ") does not exist.";
                throw std::out_of_range(ss.str());
            }
        }

    }
}
