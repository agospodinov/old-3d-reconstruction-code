#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include <memory>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class PointOfView;
            class Scene;
        }

        namespace Reconstruction
        {
            class PoseEstimator
            {
                public:
                    PoseEstimator(const std::shared_ptr<Core::Scene> &scene);

                    void EstimateCameraPose(std::shared_ptr<Core::PointOfView> &pointOfView);

                private:
                    std::shared_ptr<Core::Scene> scene;
            };
        }
    }
}

#endif // POSEESTIMATOR_H
