#include "Vision/Reconstruction/SURFGPUFeatureMatcher.h"

#include <algorithm>
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

            SURFGPUFeatureMatcher::SURFGPUFeatureMatcher(int matchNLast, int keepMLastOnGPU)
                : AbstractFeatureMatcher(matchNLast, keepMLastOnGPU),
                  imageKeypointsGPU(std::vector<cv::gpu::GpuMat>(keepMLastOnGPU)),
                  imageDescriptorsGPU(std::vector<cv::gpu::GpuMat>(keepMLastOnGPU))
            {
            }

            SURFGPUFeatureMatcher::~SURFGPUFeatureMatcher()
            {
            }

            void SURFGPUFeatureMatcher::DetectAlgorithmSpecificFeatures(int pointOfViewIndex)
            {
                const std::shared_ptr<Core::PointOfView> &pointOfView = GetPointOfView(pointOfViewIndex);
                cv::gpu::GpuMat image;

                cv::Mat newImage;
                cv::cvtColor(pointOfView->GetImage()->ToOpenCVMat(), newImage, CV_BGR2GRAY);

                image.upload(newImage);

                featureExtractor(image,
                                 cv::gpu::GpuMat(),
                                 imageKeypointsGPU[GetCurrentImageIndex() % GetKeepMLastOnGPU()],
                        imageDescriptorsGPU[GetCurrentImageIndex() % GetKeepMLastOnGPU()]);

                pointOfView->GetFeatures().resize(imageKeypointsGPU[GetCurrentImageIndex() % GetKeepMLastOnGPU()].cols);
            }

            AbstractFeatureMatcher::MatchList SURFGPUFeatureMatcher::MatchAlgorithmSpecificFeatures(int leftPOVIndex, int rightPOVIndex)
            {
                const cv::gpu::GpuMat &imageDescriptors1 = imageDescriptorsGPU[leftPOVIndex % GetKeepMLastOnGPU()];
                const cv::gpu::GpuMat &imageDescriptors2 = imageDescriptorsGPU[rightPOVIndex % GetKeepMLastOnGPU()];

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

                for (int k = 0; k < knnMatches.size(); k++)
                {
                    if (knnMatches[k][0].distance / knnMatches[k][1].distance < 0.7 /*0.6*/)
                    {
                        cv::DMatch dMatch = knnMatches[k][0];

                        cv::Mat keypointsLeft(imageKeypointsGPU[leftPOVIndex]);
                        cv::Mat keypointsRight(imageKeypointsGPU[rightPOVIndex]);

                        cv::Point2d pointInLeftImage(
                                    static_cast<double>(keypointsLeft.ptr<float>(cv::gpu::SURF_GPU::X_ROW)[dMatch.queryIdx]),
                                static_cast<double>(keypointsLeft.ptr<float>(cv::gpu::SURF_GPU::Y_ROW)[dMatch.queryIdx]));
                        cv::Point2d pointInRightImage(
                                    static_cast<double>(keypointsRight.ptr<float>(cv::gpu::SURF_GPU::X_ROW)[dMatch.trainIdx]),
                                static_cast<double>(keypointsRight.ptr<float>(cv::gpu::SURF_GPU::Y_ROW)[dMatch.trainIdx]));


                        AbstractFeatureMatcher::Match match(dMatch.queryIdx, dMatch.trainIdx, pointInLeftImage, pointInRightImage);
                        matches.push_back(match);
                    }
                }

                //        matcher.match(imageDescriptors1, imageDescriptors2, matches);


                //            cout << "KNNMatches size: " << knnMatches.size() << endl;
                // refer to the documentation of imageMatches in the header file
                std::pair<int, int> imageIndices(leftPOVIndex, rightPOVIndex);
                imageMatches[imageIndices] = matches;

                return matches;
            }

        }
    }
}
