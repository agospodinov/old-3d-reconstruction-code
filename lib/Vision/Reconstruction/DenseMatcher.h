#ifndef DENSEMATCHER_H
#define DENSEMATCHER_H

#include <memory>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Point;
            class PointOfView;
            class Scene;
        }

        namespace Reconstruction
        {
            class DenseMatcher
            {
                public:
                    DenseMatcher(const std::shared_ptr<Core::Scene> &scene);

                    void Add(const std::shared_ptr<Core::PointOfView> &pointOfView);

                private:
                    std::shared_ptr<Core::Scene> scene;
                    std::pair<std::shared_ptr<Core::PointOfView>, cv::gpu::GpuMat> lastPointOfViewWithImage;
            };

        }
    }
}

#endif // DENSEMATCHER_H
