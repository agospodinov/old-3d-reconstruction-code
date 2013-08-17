#ifndef LEVENBERGMARQUARDT_H
#define LEVENBERGMARQUARDT_H

#include <functional>

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>

#include "Math/Core/Number.h"
#include "Math/LinearAlgebra/Vector.h"

namespace Xu
{
    namespace Math
    {
        namespace Optimization
        {
            template <int Inputs = LinearAlgebra::RuntimeSized, int Outputs = LinearAlgebra::RuntimeSized>
            class LevenbergMarquardt
            {
                public:
                    LevenbergMarquardt(const std::function<Eigen::Matrix<double, Outputs, 1>(Eigen::Matrix<double, Inputs, 1>)> &function)
                        : functor(function),
                          lm(Eigen::NumericalDiff<Functor>(functor))
                    {
                    }

                    LevenbergMarquardt(const std::function<Eigen::Matrix<double, Outputs, 1>(Eigen::Matrix<double, Inputs, 1>)> &function, std::size_t inputs, std::size_t outputs)
                        : functor(inputs, outputs, function),
                          numDiff(functor),
                          lm(numDiff)
                    {
                    }

                    void Minimize(LinearAlgebra::Vector<Inputs> &data)
                    {
                        lm.minimize(data);
                    }

                private:
                    struct Functor
                    {
                            typedef double Scalar;
                            enum {
                                InputsAtCompileTime = Inputs,
                                ValuesAtCompileTime = Outputs
                            };
                            typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
                            typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
                            typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

                            const int m_inputs, m_values;
                            std::function<Eigen::Matrix<double, Outputs, 1>(Eigen::Matrix<double, Inputs, 1>)> function;

                            Functor(std::function<Eigen::Matrix<double, Outputs, 1>(Eigen::Matrix<double, Inputs, 1>)> function)
                                : m_inputs(InputsAtCompileTime),
                                  m_values(ValuesAtCompileTime),
                                  function(function)
                            {
                            }
                            Functor(int inputs, int values,
                                    std::function<Eigen::Matrix<double, Outputs, 1>(Eigen::Matrix<double, Inputs, 1>)> function)
                                : m_inputs(inputs),
                                  m_values(values),
                                  function(function)
                            {
                            }

                            int inputs() const { return m_inputs; }
                            int values() const { return m_values; }

                            int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
                            {
                                fvec = function(x);
                                return 0;
                            }
                    } functor;

                    Eigen::NumericalDiff<Functor> numDiff;
                    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor> > lm;
            };
        }
    }
}

#endif // LEVENBERGMARQUARDT_H
