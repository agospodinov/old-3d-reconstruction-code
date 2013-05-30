#include "Vision/Recognition/ShapeBasedObjectDetector.h"

#include <vector>

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "Vision/Core/PointCloud.h"

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            ShapeBasedObjectDetector::ShapeBasedObjectDetector()
            {
            }

            ShapeBasedObjectDetector::~ShapeBasedObjectDetector()
            {
            }

            std::vector<Xu::Core::Object> ShapeBasedObjectDetector::Detect(Core::VisualData data)
            {
                throw std::runtime_error("Not yet implemented");
//                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = data.GetPointCloud()->GetPCLPointCloud();
//                pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);

//                pcl::IndicesPtr indices(new std::vector<int>());
//                pcl::PassThrough<pcl::PointXYZRGB> pass;
//                pass.setInputCloud(pointCloud);
//                pass.setFilterFieldName("z");
//                pass.setFilterLimits(0.0, 1.0);
//                pass.filter(*indices);

//                pcl::RegionGrowingRGB<pcl::PointXYZRGB> segmentor;
//                segmentor.setInputCloud(pointCloud);
//                segmentor.setIndices(indices);
//                segmentor.setSearchMethod(tree);
//                segmentor.setDistanceThreshold(10);
//                segmentor.setPointColorThreshold(6);
//                segmentor.setRegionColorThreshold(5);
//                segmentor.setMinClusterSize(600);

//                std::vector<pcl::PointIndices> clusters;
//                segmentor.extract(clusters);
            }
        }
    }
}
