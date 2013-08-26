#ifndef ABSTRACTFEATUREMATCHER_H
#define ABSTRACTFEATUREMATCHER_H

#include <memory>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <opencv2/core/core.hpp>

#include "Vision/Core/Projection.h"
#include "Vision/Reconstruction/IImageMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Feature;
            class Scene;
        }

        namespace Reconstruction
        {
            class AbstractFeatureMatcher : public IImageMatcher
            {
                public:
                    AbstractFeatureMatcher(const std::shared_ptr<Core::Scene> scene, int matchNLast);
                    virtual ~AbstractFeatureMatcher();

                    virtual void ProcessImage(const std::shared_ptr<Core::PointOfView> &pointOfView);

                protected:
                    struct Match
                    {
                        Match(uint leftFeatureIndex, uint rightFeatureIndex)
                            : leftFeatureIndex(leftFeatureIndex),
                              rightFeatureIndex(rightFeatureIndex)
                        {
                        }

                        uint leftFeatureIndex;
                        uint rightFeatureIndex;
                    };

                    virtual std::vector<Core::Projection> DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView) = 0;
                    virtual std::vector<Match> MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV) = 0;

                private:
                    void CorrectMatches(std::vector<Match> &matches, std::pair<std::shared_ptr<Core::PointOfView>, std::vector<Core::Projection> > &leftPOV, std::pair<std::shared_ptr<Core::PointOfView>, std::vector<Core::Projection> > &rightPOV) const;
                    void ReconsiderOldPoints(const std::shared_ptr<Core::PointOfView> &pointOfView, std::vector<Core::Projection> detectedFeatures);

                    std::shared_ptr<Core::Scene> scene;

                    boost::circular_buffer<std::pair<std::shared_ptr<Core::PointOfView>, std::vector<Core::Projection> > > previousPointsOfView;
            };

        }
    }
}

#endif // ABSTRACTFEATUREMATCHER_H
