#ifndef FASTFEATUREMATCHER_H
#define FASTFEATUREMATCHER_H

#include <boost/circular_buffer.hpp>

#include <opencv2/features2d/features2d.hpp>

#include "Vision/Reconstruction/AbstractFeatureMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            class FASTFeatureMatcher : public AbstractFeatureMatcher
            {
                public:
                    FASTFeatureMatcher(const std::shared_ptr<Core::Scene> &scene);
                    virtual ~FASTFeatureMatcher();

                    virtual std::vector<Core::Projection> DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView);
                    virtual std::vector<Match> MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV);

                private:
                    cv::ORB featureDetector;
//                    cv::FlannBasedMatcher matcher;
                    cv::BFMatcher matcher;

                    boost::circular_buffer<std::pair<std::shared_ptr<Core::PointOfView>, cv::Mat> > imageDescriptors;

            };
        }
    }
}

#endif // FASTFEATUREMATCHER_H
