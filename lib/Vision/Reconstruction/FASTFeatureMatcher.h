#ifndef FASTFEATUREMATCHER_H
#define FASTFEATUREMATCHER_H

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
                    FASTFeatureMatcher();
                    virtual ~FASTFeatureMatcher();

                    virtual std::vector<Core::Projection> DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView);
                    virtual std::vector<Match> MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV);

            };
        }
    }
}

#endif // FASTFEATUREMATCHER_H
