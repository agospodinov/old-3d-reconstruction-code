#ifndef VISUALDATA_H
#define VISUALDATA_H

#include "Core/ObjectData.h"

namespace Xu
{
    namespace Core
    {
        class Object;
    }

    namespace Vision
    {
        namespace Core
        {
            class IImage;
            class PointCloud;

            class VisualData : public Xu::Core::ObjectData
            {
                public:
                    // TODO reimplement using new concepts.

                    VisualData(const std::shared_ptr<IImage> &image, const std::shared_ptr<PointCloud> &pointCloud);
                    VisualData(const std::shared_ptr<Xu::Core::Object> &object, const std::shared_ptr<IImage> &image, const std::shared_ptr<PointCloud> &pointCloud);
                    virtual ~VisualData();

                    const std::shared_ptr<IImage> &GetImage() const;
                    const std::shared_ptr<PointCloud> &GetPointCloud() const;

                    bool IsAssociatedWithObject() const;

                private:
                    const std::shared_ptr<IImage> image;
                    const std::shared_ptr<PointCloud> pointCloud;

                    const std::weak_ptr<Xu::Core::Object> associatedObject;
            };
        }
    }
}

#endif // VISUALDATA_H
