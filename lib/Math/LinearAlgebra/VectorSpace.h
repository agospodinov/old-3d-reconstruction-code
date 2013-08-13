#ifndef HYPERSPACE_H
#define HYPERSPACE_H

#include <array>
#include <memory>

#include <boost/array.hpp>
#include <boost/multi_array.hpp>

#include <Math/Core/Number.h>

namespace Xu
{
    namespace Math
    {
        namespace LinearAlgebra
        {
            template <std::size_t Dimensions>
            class VectorSpace
            {
                public:
                    VectorSpace(const std::array<std::size_t, Dimensions> &dimensionSizes)
                        : dimensionSizes(dimensionSizes)
                    {
                        boost::array<typename StorageType::index, Dimensions> shape = { dimensionSizes };
                        storage = std::make_shared<StorageType>(shape);
                    }

                    ~VectorSpace()
                    {

                    }

                    Math::Core::Number Get(const std::initializer_list<std::size_t> &indices) const
                    {
                        CheckDimensions(indices);

                        boost::array<typename StorageType::index, Dimensions> index = { indices };
                        return storage(index);
                    }

                    void Set(const std::initializer_list<std::size_t> &indices, const Math::Core::Number &value)
                    {
                        CheckDimensions(indices);

                        boost::array<typename StorageType::index, Dimensions> index = { indices };

                        storage(index) = value;
                    }

                    std::size_t GetDimensions() const
                    {
                        return Dimensions;
                    }

                    std::size_t GetSizeOfDimension(const std::size_t &dimension) const
                    {
                        CheckDimensionRange(dimension);
                        return dimensionSizes[dimension];
                    }

                    void Release()
                    {
                        storage.reset();
                    }

                private:

                    void CheckDimensions(const std::initializer_list<std::size_t> &indices) const
                    {
                        if (indices.size() != Dimensions)
                        {
                            std::stringstream ss;
                            ss << "Incorrect number of dimensions.";
                            throw std::out_of_range(ss.str());
                        }

                        std::size_t dimension = 0;
                        for (const std::size_t &index : indices)
                        {
                            CheckDimensionRange(dimension);
                            CheckIndexRange(dimension, index);
                            dimension++;
                        }
                    }

                    void CheckDimensionRange(const std::size_t &dimension)
                    {
                        if (dimension < 0 || dimension >= Dimensions)
                        {
                            std::stringstream ss;
                            ss << "Dimension number " << dimension << " does not exist. This hyperspace only has " << Dimensions << " dimensions.";
                            throw std::out_of_range(ss.str());
                        }
                    }

                    void CheckIndexRange(const std::size_t &dimension, const std::size_t &index) const
                    {
                        if (index < 0 || index >= dimensionSizes[dimension])
                        {
                            std::stringstream ss;
                            ss << "No element with index " << index << " found in dimension number " << dimension << ".";
                            throw std::out_of_range(ss.str());
                        }
                    }

                    std::array<std::size_t, Dimensions> dimensionSizes;

                    typedef boost::multi_array<Math::Core::Number, Dimensions> StorageType;
                    typedef std::shared_ptr<StorageType> StorageTypePtr;
                    StorageTypePtr storage;
            };
        }
    }
}
#endif // HYPERSPACE_H
