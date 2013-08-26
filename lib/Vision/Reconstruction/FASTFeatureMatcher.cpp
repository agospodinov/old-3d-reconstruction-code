#include "FASTFeatureMatcher.h"

#include <opencv2/imgproc/imgproc.hpp>

#include "Vision/Core/PointOfView.h"
#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            FASTFeatureMatcher::FASTFeatureMatcher(const std::shared_ptr<Core::Scene> &scene)
                : AbstractFeatureMatcher(scene, 2),
                  imageDescriptors(3),
                  matcher(cv::NORM_HAMMING, true)
            {

            }

            FASTFeatureMatcher::~FASTFeatureMatcher()
            {

            }

            std::vector<Core::Projection> FASTFeatureMatcher::DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                cv::Mat grayscale;
                cv::cvtColor(pointOfView->GetImage()->GetMatrix(), grayscale, CV_BGR2GRAY);
                std::vector<cv::KeyPoint> keypoints;
                cv::Mat descriptors;
                featureDetector(grayscale, cv::Mat(), keypoints, descriptors);

                imageDescriptors.push_back(std::make_pair(pointOfView, descriptors));

                std::vector<Core::Projection> projections;
                for (const cv::KeyPoint &keypoint : keypoints)
                {
                    projections.emplace_back(keypoint.pt.x, keypoint.pt.y, pointOfView, true);
                }

                return projections;
            }

            std::vector<AbstractFeatureMatcher::Match> FASTFeatureMatcher::MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV)
            {
                auto matchPOV = [](const std::shared_ptr<Core::PointOfView> &pointOfView, const std::pair<std::shared_ptr<Core::PointOfView>, cv::Mat> &pair)
                {
                    return pair.first == pointOfView;
                };

                using std::placeholders::_1;
                auto leftImageDescriptorsIt = std::find_if(imageDescriptors.begin(), imageDescriptors.end(), std::bind(matchPOV, leftPOV, _1));
                auto rightImageDescriptorsIt = std::find_if(imageDescriptors.begin(), imageDescriptors.end(), std::bind(matchPOV, rightPOV, _1));

                if (leftImageDescriptorsIt == imageDescriptors.end() || rightImageDescriptorsIt == imageDescriptors.end())
                {
                    return std::vector<AbstractFeatureMatcher::Match>();
                }

                const cv::Mat &imageDescriptors1 = leftImageDescriptorsIt->second;
                const cv::Mat &imageDescriptors2 = rightImageDescriptorsIt->second;

                if(imageDescriptors1.empty() || imageDescriptors2.empty())
                {
                    CV_Error(0, "one of the image descriptors matrices is empty");
                }

                std::vector<AbstractFeatureMatcher::Match> matches;

                std::vector<std::vector<cv::DMatch> > knnMatches;
//                std::vector<cv::DMatch> dMatches;

                matcher.knnMatch(imageDescriptors1, imageDescriptors2, knnMatches, 1);
//                matcher.match(imageDescriptors1, imageDescriptors2, dMatches);

                for (int k = 0; k < knnMatches.size(); k++)
                {
                    if (!knnMatches[k].empty())
                    {
                        const cv::DMatch &match = knnMatches[k][0];
                        matches.emplace_back(match.queryIdx, match.trainIdx);
                    }
                }

//                for (const cv::DMatch &match : dMatches)
//                {
//                    matches.emplace_back(match.queryIdx, match.trainIdx);
//                }

                return matches;
            }
        }
    }
}
