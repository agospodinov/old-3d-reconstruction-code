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

            GFTTFeatureMatcher::GFTTFeatureMatcher(int minimumFeatureThreshold, float qualityLevel)
                : AbstractFeatureMatcher(1, 0),
                  minimumFeatureThreshold(minimumFeatureThreshold),
                  qualityLevel(qualityLevel)
            {
            }

            void GFTTFeatureMatcher::DetectAlgorithmSpecificFeatures(int pointOfViewIndex)
            {
                const std::shared_ptr<Core::PointOfView> &pointOfView = GetPointOfView(pointOfViewIndex);

                currentGray.release();
                cv::cvtColor(pointOfView->GetImage()->ToOpenCVMat(), currentGray, cv::COLOR_BGR2GRAY);

                if (lastCorners.size() < minimumFeatureThreshold)
                {
                    std::vector<cv::Point2f> currentCorners;
                    cv::goodFeaturesToTrack(currentGray, currentCorners, 1000, qualityLevel, 10);
                    pointOfView->GetFeatures().resize(currentCorners.size());
                    this->currentCorners = std::move(currentCorners);
                }
                else
                {
                    status.clear();
                    errors.clear();
                    currentCorners.clear();
                    cv::calcOpticalFlowPyrLK(lastGray, currentGray, lastCorners, currentCorners, status, errors);

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
                        cv::Point2f cornerInLastImage = lastCorners.at(match.first);
                        cv::Point2f cornerInCurrentImage = currentCorners.at(match.second);

                        matches.push_back(Match(match.first,
                                                match.second,
                                                cv::Point2d(static_cast<double>(cornerInLastImage.x),
                                                            static_cast<double>(cornerInLastImage.y)),
                                                cv::Point2d(static_cast<double>(cornerInCurrentImage.x),
                                                            static_cast<double>(cornerInCurrentImage.y))));
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

//                    matches.clear();
//                    for (int i = 0; i < status.size(); i++)
//                    {
//                        if (status[i] == 1)
//                        {
//                            cv::Point2f cornerInLastImage = lastCorners.at(i);
//                            cv::Point2f cornerInCurrentImage = currentCorners.at(i);

//                            // There is a problem here... If the sizes of the current and the previous image differ
//                            // and the point is outside the boundaries of the last image, but not outside the current image
//                            // may cause an out_of_range exception to be thrown when creating features.
//                            if (cornerInLastImage.x < 0 || cornerInLastImage.x > pointOfView->GetImage()->GetSize().width ||
//                                    cornerInLastImage.y < 0 || cornerInLastImage.y > pointOfView->GetImage()->GetSize().width ||
//                                    cornerInCurrentImage.x < 0 || cornerInCurrentImage.x > pointOfView->GetImage()->GetSize().width ||
//                                    cornerInCurrentImage.y < 0 || cornerInCurrentImage.y > pointOfView->GetImage()->GetSize().height)
//                            {
//                                continue;
//                            }

//                            matches.push_back(Match(i,
//                                                    i,
//                                                    cv::Point2d(static_cast<double>(cornerInLastImage.x),
//                                                                static_cast<double>(cornerInLastImage.y)),
//                                                    cv::Point2d(static_cast<double>(cornerInCurrentImage.x),
//                                                                static_cast<double>(cornerInCurrentImage.y))));
//                        }
//                    }
                    pointOfView->GetFeatures().resize(currentCorners.size());
                }

                lastGray = currentGray;
                lastCorners = currentCorners;
            }

            AbstractFeatureMatcher::MatchList GFTTFeatureMatcher::MatchAlgorithmSpecificFeatures(int leftPOVIndex, int rightPOVIndex)
            {
                return matches;
            }

        }
    }
}
