#include "Vision/Core/PointOfView.h"

#include "Vision/Core/Feature.h"
#include "Vision/Core/ICamera.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {

            PointOfView::PointOfView(const std::shared_ptr<ICamera> &associatedCamera, std::unique_ptr<IImage> frame)
                : camera(std::weak_ptr<ICamera>(associatedCamera)),
                  image(std::move(frame))
            {
                double focalLength = std::max(GetImage()->GetSize().width, GetImage()->GetSize().height) * (3.7 / 4.54); // FIXME hardcoded values for galaxy s3 camera
                cameraParameters.SetFocalLength(focalLength);
            }

            PointOfView::~PointOfView()
            {
            }

            const std::unique_ptr<IImage> &PointOfView::GetImage() const
            {
                return image;
            }

            CameraParameters &PointOfView::GetCameraParameters()
            {
                return cameraParameters;
            }

            std::weak_ptr<ICamera> PointOfView::GetAssociatedCamera() const
            {
                return camera;
            }

        }
    }
}
