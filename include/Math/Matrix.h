#ifndef MATRIX_H
#define MATRIX_H

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>

#include <memory>

namespace Xu
{
    namespace Math
    {

        class Matrix
        {
            public:
                Matrix();
                Matrix(int rows, int columns, int channels);

                explicit Matrix(const cv::Mat &matrix);
                explicit Matrix(const Eigen::MatrixXd &matrix);

                Matrix(const Matrix &other);
                Matrix(Matrix &&other);

                ~Matrix();

                Matrix &operator= (const Matrix &other);
                Matrix &operator= (Matrix &&other);

                static Matrix Identity(int rows, int columns);
                static Matrix Zeros(int rows, int columns);
                static Matrix Ones(int rows, int columns);

                Matrix Transpose() const;

                double Get(int x, int y) const;
                double Get(int x, int y, int channel) const;

                void Set(int x, int y, double value);
                void Set(int x, int y, int channel, double value);

                size_t GetRows() const;
                size_t GetColumns() const;
                size_t GetChannels() const;

                cv::Mat ToCvMat() const;
                Eigen::MatrixXd ToEigenMatrix() const;

                void Release();

            private:
                friend std::ostream &operator<<(std::ostream &os, const Matrix &matrix);

                void CheckRange(int x, int y) const;

                std::shared_ptr<std::vector<double> > data;

                size_t rows, columns;
                size_t channels;

        };

    }
}

#endif // MATRIX_H
