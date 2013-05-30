#ifndef IFEATUREMATCHER_H
#define IFEATUREMATCHER_H

#include <memory>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class PointOfView;
        }

        namespace Reconstruction
        {
            class IImageMatcher
            {
                public:
                    virtual ~IImageMatcher();

                    virtual void AddImage(std::shared_ptr<Core::PointOfView> &pointOfView) = 0;
            };

        }
    }
}

#endif // IFEATUREMATCHER_H
