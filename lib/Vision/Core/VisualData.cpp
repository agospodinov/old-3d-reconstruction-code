#include "Vision/Core/VisualData.h"

#include "Core/Object.h"

#include "Vision/Core/IImage.h"
#include "Vision/Core/Scene.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            VisualData::VisualData(const std::shared_ptr<IImage> &image, const std::shared_ptr<PointCloud> &pointCloud)
                : image(image),
                  pointCloud(pointCloud)
            {
            }

            VisualData::VisualData(const std::shared_ptr<Xu::Core::Object> &object, const std::shared_ptr<IImage> &image, const std::shared_ptr<PointCloud> &pointCloud)
                : associatedObject(object),
                  image(image),
                  pointCloud(pointCloud)
            {
            }

            VisualData::~VisualData()
            {
            }

            const std::shared_ptr<IImage> &VisualData::GetImage() const
            {
                return image;
            }

            const std::shared_ptr<PointCloud> &VisualData::GetPointCloud() const
            {
                return pointCloud;
            }

            bool VisualData::IsAssociatedWithObject() const
            {
                return !associatedObject.expired();
            }
        }
    }
}
