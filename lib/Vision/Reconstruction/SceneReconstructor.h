#ifndef SCENERECONSTRUCTOR_H
#define SCENERECONSTRUCTOR_H

#include <vector>
#include <memory>
#include <thread>
#include <queue>
#include <future>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class FeatureSet;
            class PointOfView;
            class Scene;
            class SingleViewCamera;
        }

        namespace Reconstruction
        {
            class DenseMatcher;
            class IImageMatcher;
            class BundleAdjuster;
            class PoseEstimator;

            class SceneReconstructor
            {
                public:
                    SceneReconstructor(std::shared_ptr<Xu::Vision::Core::SingleViewCamera> camera);
                    ~SceneReconstructor();

                    bool IsInitialTriangulationReady() const;
                    std::shared_ptr<Core::Scene> GetScene() const;

                    void Run();

                private:
                    void TriangulatePoints(bool optimize = true);

                    void InitialReconstruction();

                    void AddForReconstruction(std::shared_ptr<Core::PointOfView> pointOfView);

                    bool running;
                    bool initialTriangulation;

                    std::shared_ptr<Core::Scene> scene;

                    std::unique_ptr<IImageMatcher> featureMatcher;

                    std::unique_ptr<DenseMatcher> denseMatcher;

                    std::unique_ptr<PoseEstimator> poseEstimator;

                    std::unique_ptr<BundleAdjuster> bundleAdjuster;

                    std::shared_ptr<Core::SingleViewCamera> camera;

                    std::shared_ptr<Core::PointOfView> lastPointOfView;
                    std::shared_ptr<Core::PointOfView> currentPointOfView;
            };

        }
    }
}

#endif // SCENERECONSTRUCTOR_H
