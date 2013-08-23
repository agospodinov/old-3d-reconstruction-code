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

//                    for (int i = 0; i < this->currentCorners.size(); i++)
//                    {
//                        bool isNew = true;
//                        const cv::Point2f &cornerInCurrentImage = newCorners.at(i);
//                        for (int j = 0; j < this->currentCorners.size(); j++)
//                        {
//                            const cv::Point2f &trackedCornerInCurrentImage = currentCorners.at(j);

//                            // If the OF tracked point and the newly located point are very close by
//                            // assume they are the same point.
//                            if (std::fabs(cornerInCurrentImage.x - trackedCornerInCurrentImage.x) < 5.0f &&
//                                    std::fabs(cornerInCurrentImage.y - trackedCornerInCurrentImage.y) < 5.0f)
//                            {
//                                isNew = false;
//                                break;
//                            }
//                        }

//                        if (isNew)
//                        {
//                            this->currentCorners.push_back(newCorners.at(i));
//                        }
//                    }

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
                    matches.clear();
                    // We could use the errors as a distance when adding new points for the algorithm to track
                    cv::calcOpticalFlowPyrLK(lastGray, currentGray, lastCorners, currentCorners, status, cv::noArray());
                    assert(lastCorners.size() == currentCorners.size());

                    std::size_t i, k;
                    for (i = 0, k = 0; i < currentCorners.size(); i++)
                    {
                        if (status.at(i) == 0)
                        {
                            continue;
                        }

                        cv::Point2f cornerInCurrentImage = currentCorners.at(i);

                        projections.push_back(Core::Projection(static_cast<double>(cornerInCurrentImage.x),
                                                               static_cast<double>(cornerInCurrentImage.y),
                                                               pointOfView, true));

                        matches.push_back(Match(i, k));

                        currentCorners.at(k++) = currentCorners.at(i);
                    }
                    currentCorners.resize(k);
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
