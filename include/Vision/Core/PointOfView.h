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
                    typedef std::shared_ptr<PointOfView> Ptr;
                    typedef std::weak_ptr<PointOfView> WeakPtr;

                    PointOfView(std::shared_ptr<ICamera> associatedCamera, std::unique_ptr<IImage> frame);
                    ~PointOfView();

                    const std::unique_ptr<IImage> &GetImage() const;

                    CameraParameters &GetCameraParameters();

                    std::weak_ptr<ICamera> GetAssociatedCamera() const;

                    std::vector<std::shared_ptr<Feature> > &GetFeatures();
                    void ClearFeatures();

                private:
                    std::unique_ptr<IImage> image;
                    std::vector<std::shared_ptr<Feature> > features;

                    CameraParameters cameraParameters;

                    std::weak_ptr<ICamera> camera;

            };

        }
    }
}

#endif // POINTOFVIEW_H
