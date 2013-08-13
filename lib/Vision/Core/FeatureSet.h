#ifndef FEATURESET_H
#define FEATURESET_H

#include <memory>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Feature.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {

            class Feature;
            class PointOfView;
            class IFeatureMatcher;

            class FeatureSet
            {
                public:
                    FeatureSet();
                    ~FeatureSet();

                    std::size_t Size() const;

                    Feature &CreateFeature();
                    void AddFeature(const Feature &feature);
                    void AddFeatures(const std::list<Feature> &features);

                    typename std::list<Feature>::iterator Begin();
                    typename std::list<Feature>::iterator End();

                    std::list<Feature> GetCommonPoints(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV) const;
                    std::pair<std::shared_ptr<PointOfView>, std::shared_ptr<PointOfView> > FindPairWithMostMatches() const;

                private:
                    std::list<Feature> features;

            };

        }
    }
}
#endif // FEATURESET_H
