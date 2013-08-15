#ifndef RANSAC_H
#define RANSAC_H

#include <vector>

#include "Math/Core/Number.h"
#include "Math/Calculus/IFunction.h"
#include "Math/LinearAlgebra/Vector.h"

namespace Xu
{
    namespace Math
    {
        namespace Statistics
        {
            template <int DataSpace>
            class RANSAC
            {
                public:
                    RANSAC(const std::vector<LinearAlgebra::Vector<DataSpace> > &data)
                        : data(data)
                    {
                    }


                private:
                    class Hypothesis
                    {
                        public:
                            Hypothesis(std::size_t subsetSize)
                                : indices(data.size())
                            {
                                std::iota(indices.begin(), indices.end(), 0);
                                std::random_shuffle(indices.begin(), indices.end());
                            }

                            LinearAlgebra::Vector<DataSpace> &operator [](std::size_t index)
                            {
                                return data[indices[index]];
                            }

                        private:
                            std::vector<std::size_t> indices;


                    };

                    std::vector<LinearAlgebra::Vector<DataSpace> > data;

            };
        }
    }
}

#endif // RANSAC_H
