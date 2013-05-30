#ifndef FEATURESET_H
#define FEATURESET_H

#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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

                    int Size() const;

                    void AddFeature(std::shared_ptr<Feature> feature);
                    void AddFeatures(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV);

                    void AddFeatures(const std::shared_ptr<PointOfView> &pointOfView);

                    std::shared_ptr<Feature> GetFeature(int index);
                    void RemoveFeature(int index);

                    std::vector<cv::Point3d> GetPoints3d() const;

                    std::vector<std::shared_ptr<PointOfView> > GetPointsOfView() const;

                    std::vector<cv::Point2d> GetVisiblePoints(const std::shared_ptr<PointOfView> &pointOfView) const;
                    std::vector<int> GetCommonPoints(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV, std::vector<cv::Point2d> &leftImagePoints, std::vector<cv::Point2d> &rightImagePoints) const;
                    std::pair<std::shared_ptr<PointOfView>, std::shared_ptr<PointOfView> > FindPairWithMostMatches() const;

                protected:
                    void MatchFeatures(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV, const std::vector<cv::DMatch> &matches);

                private:
                    std::vector<std::shared_ptr<Feature> > features;

                    std::vector<std::weak_ptr<PointOfView> > pointsOfView;

            };

        }
    }
}
#endif // FEATURESET_H
