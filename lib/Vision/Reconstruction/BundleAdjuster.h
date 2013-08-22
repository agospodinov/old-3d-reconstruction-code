#ifndef BUNDLEADJUSTER_H
#define BUNDLEADJUSTER_H

#include <memory>
#include <vector>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Feature;
            class FeatureSet;
            class Scene;
            class PointOfView;
        }

        namespace Reconstruction
        {
            class BundleAdjuster
            {
                public:
                    BundleAdjuster();
                    BundleAdjuster(const std::shared_ptr<Core::Scene> &scene);

                    void EstimateCameraPose(std::shared_ptr<Core::PointOfView> &pointOfView);
                    void RunOnNewData();
                    void RunOnAllData();
                    void Reset(const std::shared_ptr<Core::Scene> &scene);

                    inline void AddPOV(std::shared_ptr<Core::PointOfView> &pointOfView)
                    {
                        pointsOfView.push_back(pointOfView);
                    }

                private:

                    enum SbaMode
                    {
                        STRUCTURE_AND_MOTION,
                        STRUCTURE_ONLY,
                        MOTION_ONLY
                    };

                    struct SbaParameters
                    {
                        public:
                            SbaParameters(BundleAdjuster *instance, SbaMode mode)
                                : instance(instance),
                                  mode(mode)
                            {
                            }

                            // non-owning
                            std::vector<Core::Feature *> triangulatedPoints;
                            std::vector<std::shared_ptr<Core::PointOfView> > pointsOfView;

                            SbaMode mode;
                            BundleAdjuster *instance;
                    };


                    SbaParameters PrepareData(SbaMode mode);
                    void Run(int startingPoint, int startingCamera, SbaParameters &params);
                    static void ProjectPoint(int j, int i, double *cameraParams, double *pointParams, double *projection, void *additionalData);
                    static void ProjectPointStructureOnly(int j, int i, double *pointParams, double *projection, void *additionalData);
                    static void ProjectPointMotionOnly(int j, int i, double *cameraParams, double *projection, void *additionalData);

                    std::shared_ptr<Core::Scene> scene;
                    std::vector<std::shared_ptr<Core::PointOfView> > pointsOfView;

                    int adjustedPoints, adjustedCameras;
            };

        }
    }
}

#endif // BUNDLEADJUSTER_H
