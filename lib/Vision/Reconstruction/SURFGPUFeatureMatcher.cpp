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

            SURFGPUFeatureMatcher::SURFGPUFeatureMatcher(const std::shared_ptr<Core::Scene> &scene, int matchNLast, int keepMLastOnGPU)
                : AbstractFeatureMatcher(scene, matchNLast),
                  imageDescriptorsGPU(keepMLastOnGPU)
            {
            }

            SURFGPUFeatureMatcher::~SURFGPUFeatureMatcher()
            {
            }

            std::vector<Core::Projection> SURFGPUFeatureMatcher::DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView)
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

                cv::gpu::GpuMat imageKeypointsGPU;
                cv::gpu::GpuMat imageDescriptors;
                featureExtractor(image,
                                 cv::gpu::GpuMat(),
                                 imageKeypointsGPU,
                                 imageDescriptors);

                imageDescriptorsGPU.push_back(std::make_pair(pointOfView, imageDescriptors));

                cv::Mat imageKeypoints(imageKeypointsGPU);
                std::vector<Core::Projection> projections;

                for (int i = 0; i < imageKeypoints.cols; i++)
                {
                    projections.push_back(Core::Projection(
                                static_cast<double>(imageKeypoints.ptr<float>(cv::gpu::SURF_GPU::X_ROW)[i]),
                                static_cast<double>(imageKeypoints.ptr<float>(cv::gpu::SURF_GPU::Y_ROW)[i]),
                                pointOfView, true));
                }

                return projections;
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

                std::vector<AbstractFeatureMatcher::Match> matches;

                std::vector<std::vector<cv::DMatch> > knnMatches;
                cv::gpu::GpuMat trainIdx, distance, allDist;

                matcher.knnMatchSingle(imageDescriptors1, imageDescriptors2, trainIdx, distance, allDist, 2);
                matcher.knnMatchDownload(trainIdx, distance, knnMatches);

                for (int k = 0; k < knnMatches.size(); k++)
                {
                    if (knnMatches[k][0].distance / knnMatches[k][1].distance < 0.6 /*0.7*/)
                    {
                        const cv::DMatch &dMatch = knnMatches[k][0];

                        AbstractFeatureMatcher::Match match(dMatch.queryIdx, dMatch.trainIdx);
                        matches.push_back(match);
                    }
                }

                return matches;
            }
        }
    }
}
