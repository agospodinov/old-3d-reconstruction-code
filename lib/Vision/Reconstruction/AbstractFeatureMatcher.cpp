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

                    CorrectMatches(matches, previousPOV, currentPOV);

                    for (Match &match : matches)
                    {
                        Core::Projection &leftProjection = previousPOV.second.at(match.leftFeatureIndex);
                        Core::Projection &rightProjection = currentPOV.second.at(match.rightFeatureIndex);

                        if (leftProjection.GetAssociatedPoint() == nullptr
                                && rightProjection.GetAssociatedPoint() == nullptr)
                        {
                            Core::Feature &feature = scene->GetFeatures()->CreateFeature();
                            Core::IImage::Pixel pixel = pointOfView->GetImage()->GetPixel(rightProjection.GetX(), rightProjection.GetY());
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

            void AbstractFeatureMatcher::CorrectMatches(std::vector<Match> &matches, std::pair<std::shared_ptr<Core::PointOfView>, std::vector<Core::Projection> > &leftPOV, std::pair<std::shared_ptr<Core::PointOfView>, std::vector<Core::Projection> > &rightPOV) const
            {
                static bool first = true;

                if (matches.size() == 0)
                {
                    return;
                }

                std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
                leftImagePoints.reserve(matches.size());
                rightImagePoints.reserve(matches.size());

                for (const Match &match : matches)
                {
                    leftImagePoints.push_back(cv::Point2d(leftPOV.second.at(match.leftFeatureIndex).GetX(), leftPOV.second.at(match.leftFeatureIndex).GetY()));
                    rightImagePoints.push_back(cv::Point2d(rightPOV.second.at(match.rightFeatureIndex).GetX(), rightPOV.second.at(match.rightFeatureIndex).GetY()));
                }

                std::vector<uchar> status(matches.size());

                cv::Mat fundamentalMatrix = cv::findFundamentalMat(leftImagePoints, rightImagePoints, cv::FM_RANSAC, 3.0, 0.999, status);

                std::cout << "Inliers as determined by the fundamental matrix: " << cv::countNonZero(status) << " / " << status.size() << std::endl;

                if (cv::countNonZero(status) == 0)
                {
                    return;
                }

                std::vector<cv::Point2d> leftImageGoodPoints, rightImageGoodPoints;

                for (int i = status.size() - 1; i >= 0; i--)
                {
                    if (status.at(i) == 0) // If the point is an outlier
                    {
                        matches.erase(matches.begin() + i);
                    }
                }

                if (first)
                {
                    for (const Match &match : matches)
                    {
                        leftImageGoodPoints.push_back(cv::Point2d(leftPOV.second.at(match.leftFeatureIndex).GetX(), leftPOV.second.at(match.leftFeatureIndex).GetY()));
                        rightImageGoodPoints.push_back(cv::Point2d(rightPOV.second.at(match.rightFeatureIndex).GetX(), rightPOV.second.at(match.rightFeatureIndex).GetY()));
                    }

                    std::vector<cv::Point2d> correctedLeftImagePoints, correctedRightImagePoints;
                    cv::correctMatches(fundamentalMatrix, leftImageGoodPoints, rightImageGoodPoints, correctedLeftImagePoints, correctedRightImagePoints);

                    for (int i = 0; i < matches.size(); i++)
                    {
                        const Match &match = matches[i];
                        leftPOV.second.at(match.leftFeatureIndex).SetX(correctedLeftImagePoints.at(i).x);
                        leftPOV.second.at(match.leftFeatureIndex).SetY(correctedLeftImagePoints.at(i).y);
                        rightPOV.second.at(match.rightFeatureIndex).SetX(correctedRightImagePoints.at(i).x);
                        rightPOV.second.at(match.rightFeatureIndex).SetY(correctedRightImagePoints.at(i).y);
                    }
                }

                first = false;
            }
        }
    }
}
