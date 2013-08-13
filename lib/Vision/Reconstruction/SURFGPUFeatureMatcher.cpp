#include "Vision/Reconstruction/SURFGPUFeatureMatcher.h"

#include <algorithm>
#include <functional>
#include <utility>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Vision/Core/PointOfView.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {

            SURFGPUFeatureMatcher::SURFGPUFeatureMatcher(Core::Scene &scene, int matchNLast, int keepMLastOnGPU)
                : AbstractFeatureMatcher(scene, matchNLast),
                  imageKeypointsGPU(keepMLastOnGPU),
                  imageDescriptorsGPU(keepMLastOnGPU)
            {
            }

            SURFGPUFeatureMatcher::~SURFGPUFeatureMatcher()
            {
            }

            void SURFGPUFeatureMatcher::DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                cv::gpu::GpuMat image;

                if (pointOfView->GetImage()->HasColor())
                {
                    cv::Mat grayscale;
                    cv::cvtColor(pointOfView->GetImage()->GetMatrix(), grayscale, CV_BGR2GRAY);
                    image.upload(grayscale);
                }
                else
                {
                    image.upload(pointOfView->GetImage()->GetMatrix());
                }

                cv::gpu::GpuMat imageKeypoints;
                cv::gpu::GpuMat imageDescriptors;
                featureExtractor(image,
                                 cv::gpu::GpuMat(),
                                 imageKeypoints,
                                 imageDescriptors);

                imageKeypointsGPU.push_back(std::make_pair(pointOfView, imageKeypoints));
                imageDescriptorsGPU.push_back(std::make_pair(pointOfView, imageDescriptors));
            }

            std::vector<AbstractFeatureMatcher::Match> SURFGPUFeatureMatcher::MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV)
            {
                auto matchPOV = [](const std::shared_ptr<Core::PointOfView> &pointOfView, const std::pair<std::shared_ptr<Core::PointOfView>, cv::gpu::GpuMat> &pair)
                {
                    return pair.first == pointOfView;
                };

                using std::placeholders::_1;
                auto leftImageDescriptorsIt = std::find_if(imageDescriptorsGPU.begin(), imageDescriptorsGPU.end(), std::bind(matchPOV, leftPOV, _1));
                auto rightImageDescriptorsIt = std::find_if(imageDescriptorsGPU.begin(), imageDescriptorsGPU.end(), std::bind(matchPOV, rightPOV, _1));

                if (leftImageDescriptorsIt == imageDescriptorsGPU.end() || rightImageDescriptorsIt == imageDescriptorsGPU.end())
                {
                    return std::vector<AbstractFeatureMatcher::Match>();
                }

                const cv::gpu::GpuMat &imageDescriptors1 = leftImageDescriptorsIt->second;
                const cv::gpu::GpuMat &imageDescriptors2 = rightImageDescriptorsIt->second;

                if(imageDescriptors1.empty() || imageDescriptors2.empty())
                {
                    CV_Error(0, "one of the image descriptors matrices is empty");
                }

                // matching descriptor vectors using Brute Force matcher (only one
                // implemented in GPU module)
                cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > matcher;
                std::vector<AbstractFeatureMatcher::Match> matches;

                std::vector<std::vector<cv::DMatch> > knnMatches;
                cv::gpu::GpuMat trainIdx, distance, allDist;

                matcher.knnMatchSingle(imageDescriptors1, imageDescriptors2, trainIdx, distance, allDist, 2);
                matcher.knnMatchDownload(trainIdx, distance, knnMatches);

                cv::Mat keypointsLeft(std::find_if(imageKeypointsGPU.begin(), imageKeypointsGPU.end(), std::bind(matchPOV, leftPOV, _1))->second);
                cv::Mat keypointsRight(std::find_if(imageKeypointsGPU.begin(), imageKeypointsGPU.end(), std::bind(matchPOV, rightPOV, _1))->second);

                for (int k = 0; k < knnMatches.size(); k++)
                {
                    if (knnMatches[k][0].distance / knnMatches[k][1].distance < 0.6 /*0.7*/)
                    {
                        cv::DMatch dMatch = knnMatches[k][0];

                        Core::Projection leftProjection(
                                    static_cast<double>(keypointsLeft.ptr<float>(cv::gpu::SURF_GPU::X_ROW)[dMatch.queryIdx] - leftPOV->GetImage()->GetSize().width),
                                    static_cast<double>(keypointsLeft.ptr<float>(cv::gpu::SURF_GPU::Y_ROW)[dMatch.queryIdx] - leftPOV->GetImage()->GetSize().height),
                                    leftPOV, true);
                        Core::Projection rightProjection(
                                    static_cast<double>(keypointsRight.ptr<float>(cv::gpu::SURF_GPU::X_ROW)[dMatch.trainIdx] - rightPOV->GetImage()->GetSize().width),
                                    static_cast<double>(keypointsRight.ptr<float>(cv::gpu::SURF_GPU::Y_ROW)[dMatch.trainIdx] - rightPOV->GetImage()->GetSize().height),
                                    rightPOV, true);


                        AbstractFeatureMatcher::Match match(dMatch.queryIdx, dMatch.trainIdx, leftProjection, rightProjection);
                        matches.push_back(match);
                    }
                }

                return matches;
            }
        }
    }
}
