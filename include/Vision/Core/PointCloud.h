#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>

#include <pcl/common/common.h>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Point;
            class Feature;

            class PointCloud
            {
                public:
                    class Partial
                    {
                        public:
                            Partial(std::shared_ptr<PointCloud> pointCloud, std::vector<int> indices);

                            std::vector<Point> GetPoints() const;

                            PointCloud ToPointCloud() const;

                        private:
                            std::vector<int> indices;
                            std::weak_ptr<PointCloud> pointCloud;
                    };

                    PointCloud();

                    void AddPoint(const Point &point);
                    void ReserveSpace(const size_t &newSize);

                    int Size() const;
                    bool IsEmpty() const;

                    void Clear();

                    const std::vector<Point> &GetPoints() const;

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPCLPointCloud() const;

                private:
                    std::vector<Point> points;
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud;

            };

        }
    }
}

#endif // POINTCLOUD_H
