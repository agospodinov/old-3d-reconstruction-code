#ifndef ABSTRACTDENSEMATCHER_H
#define ABSTRACTDENSEMATCHER_H

#include "IImageMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            class AbstractDenseMatcher : public IImageMatcher
            {
                public:
                    AbstractDenseMatcher();

                    virtual void AddImage(std::shared_ptr<Core::PointOfView> &pointOfView);
            };
        }
    }
}

#endif // ABSTRACTDENSEMATCHER_H
