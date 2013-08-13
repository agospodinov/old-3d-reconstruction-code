#ifndef SCENE_H
#define SCENE_H

#include <memory>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Point;
            class PointCloud;
            class Feature;
            class FeatureSet;

            class Scene
            {
                public:
                    Scene();
                    virtual ~Scene();

                    const std::unique_ptr<FeatureSet> &GetFeatures() const;
                    const std::unique_ptr<PointCloud> &GetPointCloud() const;

                    /// remove? possibly unnecessary
                    void ResetFeatures();
                    void ResetPointCloud();
                    /// end remove


                    void UpdatePoints();
                    void ShowPointCloud();

                private:
                    std::unique_ptr<FeatureSet> features;
                    std::unique_ptr<PointCloud> pointCloud;
            };

        }
    }
}

#endif // SCENE_H
