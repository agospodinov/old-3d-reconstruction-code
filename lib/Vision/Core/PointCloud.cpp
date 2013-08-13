#include "Vision/Core/PointCloud.h"

#include "Vision/Core/Point.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {

            PointCloud::PointCloud()
//                : pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
            {
            }

            void PointCloud::AddPoint(const Point &point)
            {
                points.push_back(point);
//                pclPointCloud->push_back(point.GetPCLPoint());
            }

            void PointCloud::ReserveSpace(const size_t &newSize)
            {
                points.reserve(newSize);
//                pclPointCloud->points.reserve(newSize);
            }

            int PointCloud::Size() const
            {
                return points.size();
            }

            bool PointCloud::IsEmpty() const
            {
                return (Size() == 0);
            }

            void PointCloud::Clear()
            {
                points.clear();
//                pclPointCloud->clear();
            }

            const std::vector<Point> &PointCloud::GetPoints() const
            {
                return points;
            }

//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud::GetPCLPointCloud() const
//            {
//                return pclPointCloud;
//            }

            PointCloud::Partial::Partial(const std::shared_ptr<PointCloud> &pointCloud, const std::vector<int> &indices)
                : pointCloud(pointCloud),
                  indices(indices)
            {
            }

            std::vector<Point> PointCloud::Partial::GetPoints() const
            {
                std::vector<Point> points;
                std::shared_ptr<PointCloud> pointCloud = this->pointCloud.lock();
                if (pointCloud != NULL)
                {
                    for (int index : indices)
                    {
                        points.push_back(pointCloud->GetPoints().at(index));
                    }
                }
                return points;
            }

            PointCloud PointCloud::Partial::ToPointCloud() const
            {
                PointCloud newPointCloud;
                std::shared_ptr<PointCloud> pointCloud = this->pointCloud.lock();
                newPointCloud.ReserveSpace(indices.size());
                if (pointCloud != NULL)
                {
                    for (int index : indices)
                    {
                        newPointCloud.AddPoint(pointCloud->GetPoints().at(index));
                    }
                }
                return newPointCloud;

            }

        }
    }
}
