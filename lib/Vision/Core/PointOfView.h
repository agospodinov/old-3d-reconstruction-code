#ifndef POINTOFVIEW_H
#define POINTOFVIEW_H

#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "CameraParameters.h"
#include "IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Feature;
            class ICamera;

            class PointOfView : public std::enable_shared_from_this<PointOfView>
            {
                public:
                    PointOfView(const std::shared_ptr<ICamera> &associatedCamera, std::unique_ptr<IImage> frame);
                    ~PointOfView();

                    const std::unique_ptr<IImage> &GetImage() const;

                    const CameraParameters &GetCameraParameters() const;
                    CameraParameters &GetCameraParameters();

                    std::weak_ptr<ICamera> GetAssociatedCamera() const;

                    /// add method for clearing memory + reload image on demand
                    /// ImageProxy, anyone?

                private:
                    std::unique_ptr<IImage> image;

                    CameraParameters cameraParameters;

                    std::weak_ptr<ICamera> camera;

            };

        }
    }
}

#endif // POINTOFVIEW_H
