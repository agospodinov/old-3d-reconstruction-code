#include "Vision/Core/Feature.h"

#include "Vision/Core/PointOfView.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            Feature::Feature()
            {
            }

            Feature::~Feature()
            {
            }

            bool operator ==(const Feature &left, const Feature &right)
            {
                // FIXME implement properly
                return static_cast<Point>(left) == static_cast<Point>(right);
            }

            std::size_t hash_value(const Feature &feature)
            {
                // FIXME implement properly
            }

        }
    }
}
