#ifndef RANSAC_H
#define RANSAC_H

#include <algorithm>
#include <functional>
#include <mutex>
#include <set>
#include <thread>
#include <tuple>
#include <vector>

#include "Math/Core/Number.h"

namespace Xu
{
    namespace Math
    {
        namespace Statistics
        {
            /**
             * @brief Implements the RANSAC algorithm.
             *
             * Input:
             * U = { x_i } // set of data points
             * f(S): S -> T // a function that computes the model parameters
             * c(T, x) // the cost function
             *
             * Output:
             * T' // the best model parameters found according to the cost
             *
             */
            template <typename ValueType, typename ModelType, typename ErrorType>
            class RANSAC
            {
                public:
                    /**
                     * @brief RANSAC
                     * @param samples The number of iterations to try with
                     * different samples. If it's 0, dynamically estimate
                     * the probability of finding a good sample.
                     */
                    RANSAC(std::function<ModelType(std::vector<const ValueType *>)> fittingFunction,
                           std::function<ErrorType(ModelType, ValueType)> errorFunction,
                           std::size_t minimalSampleSize,
                           const ErrorType &errorThreshold,
                           std::size_t samples = std::numeric_limits<std::size_t>::max())
                        : fittingFunction(fittingFunction),
                          errorFunction(errorFunction),
                          minimalSampleSize(minimalSampleSize),
                          errorThreshold(errorThreshold),
                          maximumIterations(samples),
                          bestModelInliers(0),
                          bestModelError(0),
                          adaptive(samples == std::numeric_limits<std::size_t>::max())
                    {
                    }

                    ModelType Estimate(const std::vector<ValueType> &data)
                    {
                        assert (minimalSampleSize <= data.size());

                        for (int i = 0; i < maximumIterations; i++)
                        {
                            ModelType parameters;
                            std::size_t inliers;
                            ErrorType totalError;

                            std::tie(parameters, inliers, totalError) = TryHypothesis(data);

                            if (inliers > bestModelInliers || (inliers == bestModelInliers && totalError < bestModelError))
                            {
                                bestModel = parameters;
                                bestModelInliers = inliers;
                                bestModelError = totalError;
                            }

                            if (adaptive)
                            {
                                double w = static_cast<double>(inliers) / static_cast<double>(data.size());
                                std::size_t newMaximumIterations = std::log(1 - 0.99) / std::log(1 - std::pow(w, minimalSampleSize));
                                maximumIterations = std::min(maximumIterations, newMaximumIterations);
                            }
                        }

                        std::cout << "[RANSAC] Number of inliers of best model: " << bestModelInliers << std::endl;
                        std::cout << "[RANSAC] Average error of best model: " << bestModelError / static_cast<double>(bestModelInliers) << std::endl;

                        return bestModel;
                    }


                private:
                    std::tuple<ModelType, std::size_t, ErrorType> TryHypothesis(const std::vector<ValueType> &data) const
                    {
                        std::vector<std::size_t> indices(data.size());
                        std::iota(indices.begin(), indices.end(), 0);
                        std::random_shuffle(indices.begin(), indices.end());

                        std::vector<const ValueType *> sample;
                        sample.reserve(minimalSampleSize);
                        for (int j = 0; j < minimalSampleSize; j++)
                        {
                            sample.push_back(&data.at(indices[j]));
                        }

                        ModelType parameters = fittingFunction(sample);

                        std::size_t inliers = 0;
                        ErrorType totalError = 0;
                        for (const ValueType &datum : data)
                        {
                            ErrorType error = errorFunction(parameters, datum);
                            if (error < errorThreshold)
                            {
                                ++inliers;
                                totalError += error;
                            }
                        }

                        return std::make_tuple(parameters, inliers, totalError);
                    }

                    const std::size_t maxIterationsPerLevel = 32;

                    std::size_t minimalSampleSize;
                    std::function<ModelType(std::vector<const ValueType *>)> fittingFunction;
                    std::function<ErrorType(ModelType, ValueType)> errorFunction;

                    ModelType bestModel;
                    std::size_t bestModelInliers;
                    ErrorType bestModelError;

                    std::size_t maximumIterations;
                    const ErrorType errorThreshold;
                    const bool adaptive;

            };
        }
    }
}

#endif // RANSAC_H
