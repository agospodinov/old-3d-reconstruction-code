#ifndef RANSAC_H
#define RANSAC_H

#include <algorithm>
#include <functional>
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
            template <typename ValueType, typename ModelType>
            class RANSAC
            {
                public:
                    /**
                     * @brief RANSAC
                     * @param samples The number of iterations to try with
                     * different samples. If it's 0, dynamically estimate
                     * the probability of finding a good sample.
                     */
                    RANSAC(std::function<ModelType(std::vector<ValueType>)> parametersEstimationFunction,
                           std::function<Core::Number(ModelType, ValueType)> errorFunction,
                           std::size_t minimalSampleSize,
                           const Core::Number &errorThreshold,
                           std::size_t samples = std::numeric_limits<std::size_t>::max())
                        : parametersEstimationFunction(parametersEstimationFunction),
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

                        std::vector<std::size_t> indices(data.size());
                        std::iota(indices.begin(), indices.end(), 0);

                        for (int i = 0; i < maximumIterations; i++)
                        {
                            std::random_shuffle(indices.begin(), indices.end());

                            std::vector<ValueType> sample;
                            sample.reserve(minimalSampleSize);
                            for (int i = 0; i < minimalSampleSize; i++)
                            {
                                sample.push_back(data.at(indices[i]));
                            }

                            ModelType parameters = parametersEstimationFunction(sample);

                            std::size_t inliers = 0;
                            Core::Number totalError = 0;
                            for (const ValueType &datum : data)
                            {
                                Core::Number error = errorFunction(parameters, datum);
                                if (error < errorThreshold)
                                {
                                    ++inliers;
                                    totalError += error;
                                }
                            }

                            if (inliers > bestModelInliers || (inliers == bestModelInliers && totalError < bestModelError))
                            {
                                bestModel = parameters;
                                bestModelInliers = inliers;
                                bestModelError = totalError;
                            }

                            if (adaptive)
                            {
                                Core::Number w = static_cast<double>(inliers) / static_cast<double>(data.size());
                                std::size_t newMaximumIterations = std::log(1 - 0.999) / std::log(1 - std::pow(static_cast<double>(w), minimalSampleSize));
                                maximumIterations = std::min(maximumIterations, newMaximumIterations);
                            }
                        }

                        std::cout << "[RANSAC] Number of inliers of best model: " << bestModelInliers << std::endl;
                        std::cout << "[RANSAC] Average error of best model: " << bestModelError / static_cast<double>(bestModelInliers) << std::endl;

                        return bestModel;
                    }


                private:
                    std::size_t minimalSampleSize;
                    std::function<ModelType(std::vector<ValueType>)> parametersEstimationFunction;
                    std::function<Core::Number(ModelType, ValueType)> errorFunction;

                    ModelType bestModel;
                    std::size_t bestModelInliers;
                    Core::Number bestModelError;

                    Core::Number errorThreshold;
                    std::size_t maximumIterations;
                    const bool adaptive;

            };
        }
    }
}

#endif // RANSAC_H
