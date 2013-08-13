#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <memory>
#include <vector>

//#include <pcl/common/common.h>

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
                            Partial(const std::shared_ptr<PointCloud> &pointCloud, const std::vector<int> &indices);

                            /// refactor. implementations do the same thing,
                            /// just return different types of results.
                            std::vector<Point> GetPoints() const;

                            PointCloud ToPointCloud() const;
                            /// end refactor

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

//                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPCLPointCloud() const;

                private:
                    std::vector<Point> points;

                    /// remove? unnecessary, but perhaps more efficient
                    /// determine benefits and disatvantages and decide
                    /// to keep or not
//                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud;
                    /// end remove

            };

        }
    }
}

#endif // POINTCLOUD_H
