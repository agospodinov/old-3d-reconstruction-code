#include "Vision/Core/FeatureSet.h"

#include <map>
#include <set>
#include <utility>

#include "Vision/Core/Feature.h"
#include "Vision/Core/PointOfView.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            FeatureSet::FeatureSet()
            {
            }

            FeatureSet::~FeatureSet()
            {
            }

            std::size_t FeatureSet::Size() const
            {
                return features.size();
            }

            Feature &FeatureSet::CreateFeature()
            {
                features.emplace_back();
                return features.back();
            }

            void FeatureSet::AddFeature(const Feature &feature)
            {
                if (std::find(this->features.begin(), this->features.end(), feature) == this->features.end())
                {
                    this->features.push_back(feature);
                }
            }

            void FeatureSet::AddFeatures(const std::list<Feature> &features)
            {
                std::copy_if(features.begin(), features.end(), std::back_inserter(this->features), [this](const Feature &feature)
                {
                    return (std::find(this->features.begin(), this->features.end(), feature) == this->features.end());
                });
            }

            std::list<Feature>::iterator FeatureSet::Begin()
            {
                return features.begin();
            }

            std::list<Feature>::iterator FeatureSet::End()
            {
                return features.end();
            }

            std::list<Feature> FeatureSet::GetCommonPoints(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV) const
            {
                std::list<Feature> commonFeatures;
                std::copy_if(this->features.begin(), this->features.end(), std::back_inserter(commonFeatures), [&](const Feature &feature)
                {
                    return (feature.HasProjection(leftPOV) && feature.HasProjection(rightPOV));
                });
                return commonFeatures;
            }

            std::pair<std::shared_ptr<PointOfView>, std::shared_ptr<PointOfView> > FeatureSet::FindPairWithMostMatches() const
            {
                std::map<std::shared_ptr<PointOfView>, std::set<Feature> > featuresByPointOfView;
                for (const Feature &feature : features)
                {
                    for (const Projection &projection : feature.GetProjections())
                    {
                        std::shared_ptr<PointOfView> pointOfView = projection.GetPointOfView();
                        auto it = featuresByPointOfView.find(pointOfView);

                        if (it != featuresByPointOfView.end())
                        {
                            it->second.insert(feature);
                        }
                        else
                        {
                            featuresByPointOfView.insert(std::make_pair(pointOfView, std::set<Feature>({ feature })));
                        }
                    }
                }


                std::pair<std::shared_ptr<PointOfView>, std::shared_ptr<PointOfView> > mostMatchesPair;
                std::size_t highestMatchCount = 0;
                auto begin = featuresByPointOfView.begin(), end = featuresByPointOfView.end();
                // iterate all possible unique combinations
                // n! / ((n-k)!k!); k = 2
                for (auto it1 = begin; it1 != end; ++it1)
                {
                    auto it2 = it1; ++it2;
                    for (; it2 != end; ++it2)
                    {
                        std::vector<Feature> intersection;
                        std::set_intersection(it1->second.begin(), it1->second.end(), it2->second.begin(), it2->second.end(), std::back_inserter(intersection));

                        if (intersection.size() > highestMatchCount)
                        {
                            highestMatchCount = intersection.size();
                            mostMatchesPair = std::make_pair(it1->first, it2->first);
                        }
                    }
                }

                return mostMatchesPair;

                // FIXME requires implementation...
                // TODO refactor
//                std::vector<std::shared_ptr<PointOfView> > existingPointsOfView = GetPointsOfView();
//                std::pair<std::shared_ptr<PointOfView>, std::shared_ptr<PointOfView> > mostMatchesPair;
//                int highestMatchesCount = 0;

//                for (const std::shared_ptr<PointOfView> &leftPOV : existingPointsOfView)
//                {
//                    for (const std::shared_ptr<PointOfView> &rightPOV : existingPointsOfView)
//                    {
//                        if (leftPOV == rightPOV)
//                        {
//                            continue;
//                        }

//                        std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
//                        std::vector<int> featureIndices = GetCommonPoints(leftPOV, rightPOV, leftImagePoints, rightImagePoints);

//                        assert(featureIndices.size() == leftImagePoints.size() && featureIndices.size() == rightImagePoints.size());

//                        if (featureIndices.size() > highestMatchesCount)
//                        {
//                            mostMatchesPair = std::make_pair(leftPOV, rightPOV);
//                            highestMatchesCount = featureIndices.size();
//                        }
//                    }
//                }

//                std::cout << "Highest matches count: " << highestMatchesCount << std::endl;

//                return mostMatchesPair;
            }
        }
    }
}
