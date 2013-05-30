#ifndef SCENE_H
#define SCENE_H

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

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

                    const std::shared_ptr<FeatureSet> &GetFeatures() const;
                    const std::shared_ptr<PointCloud> &GetPointCloud() const;

                    void ResetFeatures();
                    void ResetPointCloud();

                    void UpdatePoints();
                    void AddPoints(std::vector<Point>);

                    //protected:
                    void ShowPointCloud();

                private:
                    std::shared_ptr<FeatureSet> features;
                    std::shared_ptr<PointCloud> pointCloud;
//                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
                    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
                    std::thread viewerThread;
                    std::mutex updateModelMutex;
                    bool update;

            };

        }
    }
}

#endif // SCENE_H
