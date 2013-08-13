#ifndef FEATURE_H
#define FEATURE_H

#include "Point.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Feature : public Point
            {
                public:
                    Feature();
                    virtual ~Feature();

                private:
                    friend bool operator ==(const Feature &left, const Feature &right);
                    friend std::size_t hash_value(const Feature &feature);

                    /// #1: due to the way features are created and matched
                    /// so far, we were using the indexes of the created
                    /// features and assigning the corresponding indexed
                    /// elements in the pov's feature list. the POV should
                    /// not own this list of features, therefore it will be
                    /// removed. now we need to figure out another way to
                    /// match features.
            };
        }
    }
}

#endif // FEATURE_H
