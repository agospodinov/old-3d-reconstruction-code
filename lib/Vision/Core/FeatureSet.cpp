#include "Vision/Core/FeatureSet.h"

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

            int FeatureSet::Size() const
            {
                return features.size();
            }

            void FeatureSet::AddFeature(std::shared_ptr<Feature> feature)
            {
                this->features.push_back(feature);
            }

            void FeatureSet::AddFeatures(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV)
            {
                std::vector<std::shared_ptr<Feature> > &leftFeatures = leftPOV->GetFeatures();
                std::vector<std::shared_ptr<Feature> > &rightFeatures = rightPOV->GetFeatures();
                //    const std::vector<cv::DMatch> &matches = featureMatcher->GetMatches(leftPOV, rightPOV);

                for (const auto &feature : leftFeatures)
                {
                    if (feature != NULL &&
                            std::find(features.begin(), features.end(), feature) == features.end())
                    {
                        features.push_back(feature);
                    }
                }

                for (const auto &feature : rightFeatures)
                {
                    if (feature != NULL &&
                            std::find(features.begin(), features.end(), feature) == features.end())
                    {
                        features.push_back(feature);
                    }
                }

                //    std::cout << "Left keypoints: " << leftFeatures.size() << std::endl;
                //    std::cout << "Right keypoints: " << rightFeatures.size() << std::endl;
                //    std::cout << "Matches: " << matches.size() << std::endl;

                //    if (matches.size() == 0)
                //    {
                //        return;
                //    }

                //    std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
                //    leftImagePoints.reserve(matches.size());
                //    rightImagePoints.reserve(matches.size());

                //    for (const DMatch &match : matches)
                //    {
                //        Size2i leftImageSize = leftPOV->GetImageSize();
                //        Size2i rightImageSize = rightPOV->GetImageSize();
                //        const vector<KeyPoint> &leftKeypoints = leftPOV->GetPointsToTrack()->GetKeypoints();
                //        const vector<KeyPoint> &rightKeypoints = rightPOV->GetPointsToTrack()->GetKeypoints();
                //        leftImagePoints.push_back(Point2d(leftKeypoints[match.queryIdx].pt.x - 0.5 * leftImageSize.width, leftKeypoints[match.queryIdx].pt.y - 0.5 * leftImageSize.height));
                //        rightImagePoints.push_back(Point2d(rightKeypoints[match.trainIdx].pt.x - 0.5 * rightImageSize.width, rightKeypoints[match.trainIdx].pt.y - 0.5 * rightImageSize.height));
                //    }

                //    std::vector<uchar> status(leftImagePoints.size());

                //    cv::Mat fundamentalMatrix = findFundamentalMat(leftImagePoints, rightImagePoints, FM_RANSAC, 3.0, 0.99, status);

                //    std::cout << "Inliers as determined by the fundamental matrix: " << countNonZero(status) << " / " << status.size() << std::endl;

                //    if (cv::countNonZero(status) == 0)
                //    {
                //        return;
                //    }

                //    std::cout << "Fundamental matrix: " << std::endl << fundamentalMatrix << std::endl << std::endl;

                //    std::vector<cv::DMatch> goodMatches;
                //    for (int i = 0; i < status.size(); i++)
                //    {
                //        if (status[i]) // If the point is an inlier
                //        {
                //            goodMatches.push_back(matches[i]);
                //        }
                //    }

                //    MatchFeatures(leftPOV, rightPOV, goodMatches);

                //    std::vector<cv::Point2d> leftImageGoodPoints, rightImageGoodPoints;
                //    std::vector<cv::Point2d> correctedLeftImagePoints, correctedRightImagePoints;

                //    std::vector<int> filteredFeatureIndices = GetCommonPoints(leftPOV, rightPOV, leftImageGoodPoints, rightImageGoodPoints);

                //    correctMatches(fundamentalMatrix, leftImageGoodPoints, rightImageGoodPoints, correctedLeftImagePoints, correctedRightImagePoints);

                //    for (int i = 0; i < filteredFeatureIndices.size(); i++)
                //    {
                //        int featureIndex = filteredFeatureIndices.at(i);
                //        std::shared_ptr<Feature> &feature = features.at(featureIndex);
                //        feature->FixCorrespondenceProjection(leftPOV, correctedLeftImagePoints[i]);
                //        feature->FixCorrespondenceProjection(rightPOV, correctedRightImagePoints[i]);
                //    }

                //    std::cout << filteredFeatureIndices.size() << " new good features added." << std::endl;
                //    std::cout << features.size() << " features total." << std::endl;
            }

            void FeatureSet::AddFeatures(const std::shared_ptr<PointOfView> &pointOfView)
            {
//                std::vector<std::shared_ptr<PointOfView> > existingPointsOfView = GetPointsOfView();
//                int nthLast = (existingPointsOfView.size() > 4) ? existingPointsOfView.size() - 4 : 0;

//                int axs = 0; // TODO rename
//                for (int i = existingPointsOfView.size() - 1; i >= 0; i--)
//                {
//                    std::shared_ptr<PointOfView> otherPointOfView = existingPointsOfView.at(i);
//                    if (otherPointOfView->GetCameraParameters().IsPoseDetermined())
//                    {
//                        AddFeatures(otherPointOfView, pointOfView);

//                        axs++;
//                    }

//                    if (axs >= 4)
//                    {
//                        break;
//                    }
//                }

//                for (int i = nthLast; i < existingPointsOfView.size(); i++)
//                {
//                    std::shared_ptr<PointOfView> otherPointOfView = existingPointsOfView.at(i);
//                    AddFeatures(otherPointOfView, pointOfView);
//                }

                for (const auto &feature : pointOfView->GetFeatures())
                {
                    if (feature != NULL &&
                            std::find(features.begin(), features.end(), feature) == features.end())
                    {
                        features.push_back(feature);
                    }
                }

                pointsOfView.push_back(pointOfView);
            }

            std::shared_ptr<Feature> FeatureSet::GetFeature(int index)
            {
                return features[index];
            }

            void FeatureSet::RemoveFeature(int index)
            {
                // TODO by index... seriously? change...
                std::vector<std::shared_ptr<Feature> >::iterator it = features.begin();
                std::advance(it, index);
                features.erase(it);
            }

            std::vector<cv::Point3d> FeatureSet::GetPoints3d() const
            {
                std::vector<cv::Point3d> points;
                for (int i = 0; i < features.size(); i++)
                {
                    std::shared_ptr<Feature> feature = features.at(i);
                    points.push_back(feature->GetPoint3d());
                }
                return points;
            }

            std::vector<std::shared_ptr<PointOfView> > FeatureSet::GetPointsOfView() const
            {
                std::vector<std::shared_ptr<PointOfView> > results;
                for (auto it : pointsOfView)
                {
                    std::shared_ptr<PointOfView> pointOfView = it.lock();
                    if (pointOfView != NULL)
                    {
                        results.push_back(pointOfView);
                    }
                }
                return results;
            }

            std::vector<int> FeatureSet::GetCommonPoints(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV, std::vector<cv::Point2d> &leftPoints, std::vector<cv::Point2d> &rightPoints) const
            {
                std::vector<int> featureIndices;
                for (int i = 0; i < features.size(); i++)
                {
                    std::shared_ptr<Feature> feature = features.at(i);

                    if (feature->HasCorrespondenceInView(leftPOV) && feature->HasCorrespondenceInView(rightPOV) && !feature->IsHidden())
                    {
                        leftPoints.push_back(feature->GetPointInView(leftPOV));
                        rightPoints.push_back(feature->GetPointInView(rightPOV));
                        featureIndices.push_back(i);
                    }
                }

                return featureIndices;
            }

            std::pair<std::shared_ptr<PointOfView>, std::shared_ptr<PointOfView> > FeatureSet::FindPairWithMostMatches() const
            {
                std::vector<std::shared_ptr<PointOfView> > existingPointsOfView = GetPointsOfView();
                std::pair<std::shared_ptr<PointOfView>, std::shared_ptr<PointOfView> > mostMatchesPair;
                int highestMatchesCount = 0;

                for (const std::shared_ptr<PointOfView> &leftPOV : existingPointsOfView)
                {
                    for (const std::shared_ptr<PointOfView> &rightPOV : existingPointsOfView)
                    {
                        if (leftPOV == rightPOV)
                        {
                            continue;
                        }

                        std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
                        std::vector<int> featureIndices = GetCommonPoints(leftPOV, rightPOV, leftImagePoints, rightImagePoints);

                        assert(featureIndices.size() == leftImagePoints.size() && featureIndices.size() == rightImagePoints.size());

                        if (featureIndices.size() > highestMatchesCount)
                        {
                            mostMatchesPair = std::make_pair(leftPOV, rightPOV);
                            highestMatchesCount = featureIndices.size();
                        }
                    }
                }

                std::cout << "Highest matches count: " << highestMatchesCount << std::endl;

                return mostMatchesPair;
            }

            void FeatureSet::MatchFeatures(const std::shared_ptr<PointOfView> &leftPOV, const std::shared_ptr<PointOfView> &rightPOV, const std::vector<cv::DMatch> &matches)
            {
                std::vector<std::shared_ptr<PointOfView> > existingPointsOfView = GetPointsOfView();
                std::vector<std::shared_ptr<Feature> > &leftImageFeatures = leftPOV->GetFeatures();
                std::vector<std::shared_ptr<Feature> > &rightImageFeatures = rightPOV->GetFeatures();

                for (const cv::DMatch &match : matches)
                {
                    std::shared_ptr<Feature> leftFeature = leftImageFeatures[match.queryIdx];
                    std::shared_ptr<Feature> rightFeature = rightImageFeatures[match.trainIdx];
                    if (leftFeature == NULL || rightFeature == NULL || leftFeature == rightFeature/* || *leftFeature == *rightFeature*/)
                    {
                        continue;
                    }

                    Feature mergedFeature = Feature::Merge(*leftFeature, *rightFeature);

                    for (int i = 0; i < features.size(); i++)
                    {
                        if (features[i] == leftFeature)
                        {
                            RemoveFeature(i);
                        }
                    }
                    for (int i = 0; i < features.size(); i++)
                    {
                        if (features[i] == rightFeature)
                        {
                            RemoveFeature(i);
                        }
                    }

                    std::shared_ptr<Feature> newFeature = std::make_shared<Feature>(mergedFeature);

                    for (std::shared_ptr<PointOfView> pointOfView : existingPointsOfView)
                    {
                        if (newFeature->HasCorrespondenceInView(pointOfView))
                        {
                            int index = newFeature->GetPointIndexInView(pointOfView);
                            std::vector<std::shared_ptr<Feature> > &features = pointOfView->GetFeatures();
                            features[index] = newFeature;
                        }
                    }

                    leftImageFeatures[match.queryIdx] = newFeature;
                    rightImageFeatures[match.trainIdx] = newFeature;

                    features.push_back(newFeature);
                }
            }

            std::vector<cv::Point2d> FeatureSet::GetVisiblePoints(const std::shared_ptr<PointOfView> &pointOfView) const
            {
                std::vector<cv::Point2d> imagePoints;
                for (std::shared_ptr<Feature> feature : features)
                {
                    if (feature->HasCorrespondenceInView(pointOfView))
                    {
                        cv::Point2d imagePoint = feature->GetPointInView(pointOfView);
                        imagePoints.push_back(imagePoint);
                    }
                }
                return imagePoints;
            }

        }
    }
}
