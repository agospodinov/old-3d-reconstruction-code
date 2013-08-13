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
                    AbstractFeatureMatcher(Core::Scene &scene, int matchNLast);
                    virtual ~AbstractFeatureMatcher();

                    virtual void ProcessImage(const std::shared_ptr<Core::PointOfView> &pointOfView);

                protected:
                    struct Match
                    {
                        Match(uint leftFeatureIndex, uint rightFeatureIndex, const Core::Projection &leftPoint, const Core::Projection &rightPoint)
                            : leftFeatureIndex(leftFeatureIndex),
                              rightFeatureIndex(rightFeatureIndex),
                              leftPoint(leftPoint),
                              rightPoint(rightPoint)
                        {
                        }

                        uint leftFeatureIndex;
                        uint rightFeatureIndex;
                        Core::Projection leftPoint;
                        Core::Projection rightPoint;
                    };

                    virtual void DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView) = 0;
                    virtual std::vector<Match> MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV) = 0;

                private:
                    void CorrectMatches(std::vector<Match> &matches, const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV) const;

                    Core::Scene * const scene;

                    /**
                     * @brief The last N points of view.
                     *
                     * The index of the n'th last point of view is
                     * nthLastImageIndex % matchNLast and the current one is
                     * currentImageIndex % matchNLast.
                     *
                     * @note This class does not own the points of view,
                     * therefore it is safe to keep pointers to them.
                     */
                    boost::circular_buffer<std::shared_ptr<Core::PointOfView> > previousPointsOfView;
            };

        }
    }
}

#endif // ABSTRACTFEATUREMATCHER_H
