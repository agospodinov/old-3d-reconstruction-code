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
                    void RejectBadPoints(const std::shared_ptr<Core::PointOfView> &pointOfView, const std::vector<int> &featureIndices, const std::vector<uchar> &status);

                    void InitialReconstruction();
                    bool EstimateCameraPose(const std::shared_ptr<Core::PointOfView> &pointOfView, cv::Mat &rotationMatrix, cv::Mat &translationMatrix);

                    void AddForReconstruction(std::shared_ptr<Core::PointOfView> pointOfView);

                    void RunFeatureMatching();
                    void RunReconstruction();
                    void RunBundleAdjustment();

                    bool running;
                    bool initialTriangulation;
                    bool shouldRunBundleAdjustment;

                    std::thread featureMatchingThread;
                    std::thread reconstructionThread;
                    std::thread bundleAdjustmentThread;

                    std::unique_ptr<DenseMatcher> denseMatcher;

                    std::unique_ptr<BundleAdjuster> bundleAdjuster;

                    std::shared_ptr<Core::FeatureSet> features;

                    std::shared_ptr<Core::SingleViewCamera> camera;

                    std::unique_ptr<IImageMatcher> featureMatcher;

                    std::queue<std::shared_ptr<Core::PointOfView> > matchedPointsOfView;
                    std::mutex pointOfViewQueueMutex;

                    std::shared_ptr<Core::PointOfView> lastPointOfView;
                    std::shared_ptr<Core::PointOfView> currentPointOfView;

                    std::shared_ptr<Core::Scene> scene;
            };

        }
    }
}

#endif // SCENERECONSTRUCTOR_H
