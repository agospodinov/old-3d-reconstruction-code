#include "Vision/Reconstruction/AbstractFeatureMatcher.h"

#include <thread>

#include <boost/circular_buffer.hpp>

#include <opencv2/calib3d/calib3d.hpp>

#include "Vision/Core/Feature.h"
#include "Vision/Core/FeatureSet.h"
#include "Vision/Core/Point.h"
#include "Vision/Core/PointOfView.h"
#include "Vision/Core/Projection.h"
#include "Vision/Core/SingleViewImage.h"
#include "Vision/Core/Scene.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            AbstractFeatureMatcher::AbstractFeatureMatcher(Core::Scene &scene, int matchNLast)
                : scene(&scene),
                  previousPointsOfView(matchNLast)
            {
            }

            AbstractFeatureMatcher::~AbstractFeatureMatcher()
            {
            }

            void AbstractFeatureMatcher::ProcessImage(const std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                std::vector<Core::Projection> currentProjections = DetectAlgorithmSpecificFeatures(pointOfView);

                auto currentPOV = std::make_pair(pointOfView, currentProjections);

                for (auto &previousPOV : previousPointsOfView)
                {
                    std::vector<Match> matches = MatchAlgorithmSpecificFeatures(previousPOV.first, currentPOV.first);

                    for (Match &match : matches)
                    {
                        Core::Projection &leftProjection = previousPOV.second.at(match.leftFeatureIndex);
                        Core::Projection &rightProjection = currentPOV.second.at(match.rightFeatureIndex);

                        if (leftProjection.GetAssociatedPoint() == nullptr
                                && rightProjection.GetAssociatedPoint() == nullptr)
                        {
                            Core::Feature &feature = scene->GetFeatures()->CreateFeature();
                            Core::IImage::Pixel pixel = pointOfView->GetImage()->GetPixel(rightProjection.GetX() + pointOfView->GetImage()->GetSize().width,
                                                                                          rightProjection.GetY() + pointOfView->GetImage()->GetSize().height);
                            feature.SetColor(pixel.red, pixel.green, pixel.blue);

                            leftProjection.SetAssociatedPoint(&feature);
                            rightProjection.SetAssociatedPoint(&feature);
                            feature.AddProjection(leftProjection);
                            feature.AddProjection(rightProjection);
                        }
                        else if (leftProjection.GetAssociatedPoint() != nullptr
                                 && rightProjection.GetAssociatedPoint() == nullptr)
                        {
                            Core::Point *point = leftProjection.GetAssociatedPoint();
                            rightProjection.SetAssociatedPoint(point);
                            point->AddProjection(rightProjection);
                        }
                        else if (leftProjection.GetAssociatedPoint() == nullptr
                                 && rightProjection.GetAssociatedPoint() != nullptr)
                        {
                            Core::Point *point = rightProjection.GetAssociatedPoint();
                            leftProjection.SetAssociatedPoint(point);
                            point->AddProjection(leftProjection);
                        }
                        else if (leftProjection.GetAssociatedPoint() != nullptr
                                 && rightProjection.GetAssociatedPoint() != nullptr
                                 && leftProjection.GetAssociatedPoint() != rightProjection.GetAssociatedPoint())
                        {
                            std::cout << "Different points. Need to merge." << std::endl;
                        }
                    }
                }

                previousPointsOfView.push_back(currentPOV);
            }

            void AbstractFeatureMatcher::CorrectMatches(std::vector<Match> &matches, const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV) const
            {
//                if (matches.size() == 0)
//                {
//                    return;
//                }

//                std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
//                leftImagePoints.reserve(matches.size());
//                rightImagePoints.reserve(matches.size());

//                for (const Match &match : matches)
//                {
//                    Core::IImage::Size leftImageSize = leftPOV.GetImage()->GetSize();
//                    Core::IImage::Size rightImageSize = rightPOV.GetImage()->GetSize();
//                    leftImagePoints.push_back(cv::Point2d(match.leftPoint.x - 0.5 * leftImageSize.width, match.leftPoint.y - 0.5 * leftImageSize.height));
//                    rightImagePoints.push_back(cv::Point2d(match.rightPoint.x - 0.5 * rightImageSize.width, match.rightPoint.y - 0.5 * rightImageSize.height));
//                }

//                std::vector<uchar> status(leftImagePoints.size());

//                cv::Mat fundamentalMatrix = cv::findFundamentalMat(leftImagePoints, rightImagePoints, cv::FM_RANSAC, 3.0, 0.99, status);

//                std::cout << "Inliers as determined by the fundamental matrix: " << cv::countNonZero(status) << " / " << status.size() << std::endl;

//                if (cv::countNonZero(status) == 0)
//                {
//                    return;
//                }

//                //    std::cout << "Fundamental matrix: " << std::endl << fundamentalMatrix << std::endl << std::endl;

//                std::vector<cv::Point2d> leftImageGoodPoints, rightImageGoodPoints;
//                std::vector<cv::Point2d> correctedLeftImagePoints, correctedRightImagePoints;

//                for (int i = status.size() - 1; i > 0; i--)
//                {
//                    if (status.at(i) == 0) // If the point is an outlier
//                    {
//                        matches.erase(matches.begin() + i);
//                    }
//                }

//                for (const Match &match : matches)
//                {
//                    Core::IImage::Size leftImageSize = leftPOV->GetImage()->GetSize();
//                    Core::IImage::Size rightImageSize = rightPOV->GetImage()->GetSize();
//                    leftImageGoodPoints.push_back(cv::Point2d(match.leftPoint.x - 0.5 * leftImageSize.width, match.leftPoint.y - 0.5 * leftImageSize.height));
//                    rightImageGoodPoints.push_back(cv::Point2d(match.rightPoint.x - 0.5 * rightImageSize.width, match.rightPoint.y - 0.5 * rightImageSize.height));
//                }

//                cv::correctMatches(fundamentalMatrix, leftImageGoodPoints, rightImageGoodPoints, correctedLeftImagePoints, correctedRightImagePoints);

//                for (int i = 0; i < matches.size(); i++)
//                {
//                    Match &match = matches[i];
//                    Core::IImage::Size leftImageSize = leftPOV->GetImage()->GetSize();
//                    Core::IImage::Size rightImageSize = rightPOV->GetImage()->GetSize();
//                    match.leftPoint = cv::Point2d(correctedLeftImagePoints.at(i).x + 0.5 * leftImageSize.width, correctedLeftImagePoints.at(i).y + 0.5 * leftImageSize.height);
//                    match.rightPoint = cv::Point2d(correctedRightImagePoints.at(i).x + 0.5 * rightImageSize.width, correctedRightImagePoints.at(i).y + 0.5 * rightImageSize.height);
//                }
            }

//            void AbstractFeatureMatcher::CreateAndMatchFeatures()
//            {
//                //#pragma omp parallel for
//                for (int previousPOVIndex = GetNthLastImageIndex(); previousPOVIndex < GetCurrentImageIndex(); previousPOVIndex++)
//                {
//                    int currentPOVIndex = GetCurrentImageIndex();

//                    std::shared_ptr<Core::PointOfView> &previousPOV = GetPointOfView(previousPOVIndex);
//                    std::shared_ptr<Core::PointOfView> &currentPOV = GetPointOfView(currentPOVIndex);

//                    std::vector<std::shared_ptr<Core::Feature> > &leftImageFeatures = previousPOV->GetFeatures();
//                    std::vector<std::shared_ptr<Core::Feature> > &rightImageFeatures = currentPOV->GetFeatures();

//                    MatchList matches = MatchAlgorithmSpecificFeatures(previousPOVIndex, currentPOVIndex);

//                    CorrectMatches(matches, previousPOV, currentPOV);

//                    for (const Match &match : matches)
//                    {
//                        std::shared_ptr<Core::Feature> leftFeature = leftImageFeatures.at(match.leftFeatureIndex);
//                        std::shared_ptr<Core::Feature> rightFeature = rightImageFeatures.at(match.rightFeatureIndex);

//                        if (leftFeature != NULL && rightFeature != NULL)
//                        {
//                            // FIXME
//                            Core::Feature mergedFeature = Core::Feature::Merge(*leftFeature, *rightFeature);

//                            std::shared_ptr<Core::Feature> newFeature = std::make_shared<Core::Feature>(mergedFeature);

//                            // FIXME
//                            for (const auto &it: newFeature->GetCorrespondeces())
//                            {
//                                std::shared_ptr<Core::PointOfView> pointOfView = it.first.lock();
//                                // FIXME
//                                if (newFeature->HasCorrespondenceInView(pointOfView))
//                                {
//                                    // FIXME
//                                    int index = newFeature->GetPointIndexInView(pointOfView);
//                                    std::vector<std::shared_ptr<Core::Feature> > &features = pointOfView->GetFeatures();
//                                    features[index] = newFeature;
//                                }
//                            }

//                            leftImageFeatures[match.leftFeatureIndex] = newFeature;
//                            rightImageFeatures[match.rightFeatureIndex] = newFeature;
//                        }
//                        else if (leftFeature != NULL && rightFeature == NULL)
//                        {
//                            // FIXME
//                            leftFeature->AddCorrespondence(currentPOV, match.rightFeatureIndex, match.rightPoint);
//                            rightImageFeatures[match.rightFeatureIndex] = leftFeature;
//                        }
//                        else if (leftFeature == NULL && rightFeature != NULL)
//                        {
//                            // FIXME
//                            rightFeature->AddCorrespondence(previousPOV, match.leftFeatureIndex, match.leftPoint);
//                            leftImageFeatures[match.leftFeatureIndex] = rightFeature;
//                        }
//                        else
//                        {
//                            std::shared_ptr<Core::Feature> newFeature = CreateFeature(previousPOV, currentPOV, match);

//                            leftImageFeatures[match.leftFeatureIndex] = newFeature;
//                            rightImageFeatures[match.rightFeatureIndex] = newFeature;
//                        }
//                    }
//                }
//            }

        }
    }
}
