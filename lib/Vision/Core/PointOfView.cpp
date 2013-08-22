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
                Math::LinearAlgebra::Matrix<3, 3> cameraMatrix;
                cameraMatrix <<
                        focalLength,           0, GetImage()->GetSize().width  / 2,
                                  0, focalLength, GetImage()->GetSize().height / 2,
                                  0,           0,                                1;


                cameraParameters.SetCameraMatrix(cameraMatrix);
            }

            PointOfView::PointOfView(const PointOfView &other)
                : camera(other.camera),
                  cameraParameters(other.cameraParameters),
                  image(other.image->Copy())
            {
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
