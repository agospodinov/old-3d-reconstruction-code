#include "Vision/Core/Scene.h"

#include "Vision/Core/Point.h"
#include "Vision/Core/PointCloud.h"
#include "Vision/Core/Feature.h"
#include "Vision/Core/FeatureSet.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            Scene::Scene()
                : pointCloud(std::unique_ptr<PointCloud>(new PointCloud())),
                  features(std::unique_ptr<FeatureSet>(new FeatureSet()))
            {
            }

            Scene::~Scene()
            {
            }

            void Scene::ResetFeatures()
            {
                features = std::unique_ptr<FeatureSet>(new FeatureSet());
            }

            void Scene::ResetPointCloud()
            {
                pointCloud = std::unique_ptr<PointCloud>(new PointCloud());
            }

            const std::unique_ptr<FeatureSet> &Scene::GetFeatures() const
            {
                return features;
            }

            const std::unique_ptr<PointCloud> &Scene::GetPointCloud() const
            {
                return pointCloud;
            }
        }
    }
}
