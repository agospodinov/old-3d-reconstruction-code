#include "Vision/Reconstruction/GFTTFeatureMatcher.h"
#include <opencv2/video/tracking.hpp>

#include "Vision/Core/PointOfView.h"
#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {

            GFTTFeatureMatcher::GFTTFeatureMatcher(Core::Scene &scene, int minimumFeatureThreshold, float qualityLevel)
                : AbstractFeatureMatcher(scene, 1),
                  minimumFeatureThreshold(minimumFeatureThreshold),
                  qualityLevel(qualityLevel)
            {
            }

            std::vector<Core::Projection> GFTTFeatureMatcher::DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                std::vector<Core::Projection> projections;

                currentGray.release();
                cv::cvtColor(pointOfView->GetImage()->GetMatrix(), currentGray, cv::COLOR_BGR2GRAY);

                if (lastCorners.size() < minimumFeatureThreshold)
                {
                    std::vector<cv::Point2f> currentCorners;
                    cv::goodFeaturesToTrack(currentGray, currentCorners, 1000, qualityLevel, 10);
                    this->currentCorners = std::move(currentCorners);

                    for (const cv::Point2f &point : this->currentCorners)
                    {
                        projections.push_back(Core::Projection(static_cast<double>(point.x),
                                                               static_cast<double>(point.y),
                                                               pointOfView, true));
                    }
                }
                else
                {
                    std::vector<uchar> status;
                    currentCorners.clear();
                    // We could use the errors as a distance when adding new points for the algorithm to track
                    cv::calcOpticalFlowPyrLK(lastGray, currentGray, lastCorners, currentCorners, status, cv::noArray());

                    std::vector<std::pair<int, int> > matchesx; // FIXME rename
                    int index = 0;
                    int keepIndex = 0;
                    auto shouldRemove = [&](const cv::Point2f &point) {
                        if (status[index] == 0)
                        {
                            index++;
                            return true;
                        }
                        else if (point.x < 0 || point.x > pointOfView->GetImage()->GetSize().width ||
                                 point.y < 0 || point.y > pointOfView->GetImage()->GetSize().height)
                        {
                            index++;
                            return true;
                        }
                        matchesx.push_back(std::make_pair(index, keepIndex));
                        index++;
                        keepIndex++;
                        return false;
                    };

                    currentCorners.erase(std::remove_if(currentCorners.begin(), currentCorners.end(), shouldRemove), currentCorners.end());

                    matches.clear();
                    for (const std::pair<int, int> &match : matchesx)
                    {
                        cv::Point2f cornerInCurrentImage = currentCorners.at(match.second);

                        projections.push_back(Core::Projection(static_cast<double>(cornerInCurrentImage.x),
                                                               static_cast<double>(cornerInCurrentImage.y),
                                                               pointOfView, true));

                        matches.push_back(Match(match.first, match.second));
                    }

                    if (currentCorners.size() < minimumFeatureThreshold)
                    {
                        std::vector<cv::Point2f> newCorners;
                        cv::goodFeaturesToTrack(currentGray, newCorners, 1000, qualityLevel, 10);
                        for (int i = 0; i < newCorners.size(); i++)
                        {
                            bool isNew = true;
                            const cv::Point2f &cornerInCurrentImage = newCorners.at(i);
                            for (int j = 0; j < this->currentCorners.size(); j++)
                            {
                                const cv::Point2f &trackedCornerInCurrentImage = currentCorners.at(j);

                                // If the OF tracked point and the newly located point are very close by
                                // assume they are the same point.
                                if (std::fabs(cornerInCurrentImage.x - trackedCornerInCurrentImage.x) < 5.0f &&
                                        std::fabs(cornerInCurrentImage.y - trackedCornerInCurrentImage.y) < 5.0f)
                                {
                                    isNew = false;
                                }
                            }

                            if (isNew)
                            {
                                currentCorners.push_back(newCorners.at(i));
                            }
                        }
                    }
                }

                lastGray = currentGray;
                lastCorners = currentCorners;
                lastPOV = pointOfView;

                return projections;
            }

            std::vector<AbstractFeatureMatcher::Match> GFTTFeatureMatcher::MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV)
            {
                return matches;
            }

        }
    }
}
