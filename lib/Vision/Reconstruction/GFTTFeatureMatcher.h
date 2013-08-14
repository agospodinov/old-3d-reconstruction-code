#ifndef GFTTFEATUREMATCHER_H
#define GFTTFEATUREMATCHER_H

#include "AbstractFeatureMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            class GFTTFeatureMatcher : public AbstractFeatureMatcher
            {
                public:
                    GFTTFeatureMatcher(Core::Scene &scene, int minimumFeatureThreshold = 45, float qualityLevel = 0.1);

                    virtual std::vector<Core::Projection> DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView);
                    virtual std::vector<Match> MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV);

                private:
                    const int minimumFeatureThreshold;
                    const float qualityLevel;

                    std::shared_ptr<Core::PointOfView> lastPOV;

                    std::vector<cv::Point2f> lastCorners;
                    std::vector<cv::Point2f> currentCorners;

                    cv::Mat lastGray;
                    cv::Mat currentGray;

                    std::vector<Match> matches;

            };

        }
    }
}
#endif // GFTTFEATUREMATCHER_H
