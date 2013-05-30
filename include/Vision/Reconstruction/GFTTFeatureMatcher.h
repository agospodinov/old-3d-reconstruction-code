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
                    GFTTFeatureMatcher(int minimumFeatureThreshold = 45, float qualityLevel = 0.1);

                    virtual void DetectAlgorithmSpecificFeatures(int pointOfViewIndex);
                    virtual MatchList MatchAlgorithmSpecificFeatures(int leftPOVIndex, int rightPOVIndex);

                private:
                    const int minimumFeatureThreshold;
                    const float qualityLevel;

                    std::vector<cv::Point2f> lastCorners;
                    std::vector<cv::Point2f> currentCorners;

                    cv::Mat lastGray;
                    cv::Mat currentGray;
                    std::vector<uchar> status;
                    std::vector<float> errors;

                    std::vector<Match> matches;

            };

        }
    }
}
#endif // GFTTFEATUREMATCHER_H
