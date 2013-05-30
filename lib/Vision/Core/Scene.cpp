#include "Vision/Core/Scene.h"

#include <pcl/visualization/pcl_visualizer.h>

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
                : pointCloud(std::make_shared<PointCloud>()),
                  features(std::make_shared<FeatureSet>()),
                  viewerThread(std::thread(std::bind(&Scene::ShowPointCloud, this))),
                  update(false)
            {
            }

            Scene::~Scene()
            {
                viewer->close();
                viewerThread.join();
            }

            void Scene::UpdatePoints()
            {
                std::lock_guard<std::mutex> lock(updateModelMutex);
//                pointCloud->Clear();
//                for (int i = 0; i < features->Size(); i++)
//                {
//                    const std::shared_ptr<Core::Feature> &feature = features->GetFeature(i);
//                    pointCloud->AddPoint(*(feature.get()));
//                }
                update = true;
            }

            void Scene::ResetFeatures()
            {
                features = std::make_shared<FeatureSet>();
            }

            void Scene::ResetPointCloud()
            {
                pointCloud = std::make_shared<PointCloud>();
            }

            const std::shared_ptr<FeatureSet> &Scene::GetFeatures() const
            {
                return features;
            }

            const std::shared_ptr<PointCloud> &Scene::GetPointCloud() const
            {
                return pointCloud;
            }

            void Scene::AddPoints(std::vector<Point> points)
            {
                for (int i = 0; i < points.size(); i++)
                {
                    Point point = points.at(i);
                    if (point.IsTriangulated() && !point.IsHidden())
                    {
                        pointCloud->AddPoint(point);
                    }
                }
            }

            // This method doesn't belong here, but in some visualization logic instead...
            void Scene::ShowPointCloud()
            {
                while (pointCloud == NULL || pointCloud.get() == NULL || pointCloud->IsEmpty())
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(100000));
                }

                viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D viewer");
                viewer->setBackgroundColor(0.1, 0.1, 0.1);
                viewer->addPointCloud(pointCloud->GetPCLPointCloud(), "sample cloud");
                //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
                viewer->addCoordinateSystem(1.0);
                viewer->initCameraParameters();

                while (!viewer->wasStopped())
                {
                    // Get lock on the boolean update and check if cloud was updated
                    {
                        std::lock_guard<std::mutex> updateLock(updateModelMutex);
                        if (update)
                        {
                            viewer->updatePointCloud(pointCloud->GetPCLPointCloud(), "sample cloud");
                            update = false;
                        }
                    }
                    viewer->spinOnce(100);
                }
            }

        }
    }
}
