#include "Vision/Reconstruction/DenseMatcher.h"

#include <utility>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/gpu/gpu.hpp>

#include "Vision/Core/Point.h"
#include "Vision/Core/PointOfView.h"
#include "Vision/Core/PointCloud.h"
#include "Vision/Core/Scene.h"
#include "Vision/Core/FeatureSet.h"
#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {

            DenseMatcher::DenseMatcher(const std::shared_ptr<Core::Scene> &scene)
                : scene(scene)
            {
            }

            void DenseMatcher::Add(const std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                cv::Mat image = pointOfView->GetImage()->GetMatrix();
                cv::Mat gray;

                double scaleFactor = 800.0 / static_cast<double>(std::max(image.rows, image.cols));
                cv::resize(image, image, cv::Size(), scaleFactor, scaleFactor);
                image.convertTo(image, CV_32F, 1.0 / 255.0);
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                cv::gpu::GpuMat imageGPU; imageGPU.upload(gray);
                if (lastPointOfViewWithImage.first == NULL)
                {
                    lastPointOfViewWithImage = std::make_pair(pointOfView, imageGPU);
                    return;
                }

                std::pair<std::shared_ptr<Core::PointOfView>, cv::gpu::GpuMat> currentPointOfViewWithImage;
                currentPointOfViewWithImage = std::make_pair(pointOfView, imageGPU);

                // TODO allow changing these parameters
                float scale = 0.8;
                float alpha = 0.197;
                float gamma = 50.0;
                int inner = 10;
                int outer = 77;
                int solver = 10;

                std::cout << "Estimating optical flow... " << std::flush;

                cv::Mat forwardHorizontalFlow, forwardVerticalFlow;
                cv::Mat backwardHorizontalFlow, backwardVerticalFlow;

                #define BROX_OVER_TVL1
                #ifdef BROX_OVER_TVL1
                {
                    cv::gpu::BroxOpticalFlow flow(alpha, gamma, scale, inner, outer, solver);
                    cv::gpu::GpuMat horizontalFlowGPU, verticalFlowGPU;

                    std::cout << "Forward... " << std::flush;
                    flow(lastPointOfViewWithImage.second, currentPointOfViewWithImage.second, horizontalFlowGPU, verticalFlowGPU);
                    horizontalFlowGPU.download(forwardHorizontalFlow);
                    verticalFlowGPU.download(forwardVerticalFlow);

                    std::cout << "Backward... " << std::flush;
                    flow(currentPointOfViewWithImage.second, lastPointOfViewWithImage.second, horizontalFlowGPU, verticalFlowGPU);
                    horizontalFlowGPU.download(backwardHorizontalFlow);
                    verticalFlowGPU.download(backwardVerticalFlow);
                }
                #else
                {
                    cv::gpu::OpticalFlowDual_TVL1_GPU flow;
                    cv::gpu::GpuMat horizontalFlowGPU, verticalFlowGPU;

                    std::cout << "Forward... " << std::flush;
                    flow(lastPointOfViewWithImage.second, currentPointOfViewWithImage.second, horizontalFlowGPU, verticalFlowGPU);
                    horizontalFlowGPU.download(forwardHorizontalFlow);
                    verticalFlowGPU.download(forwardVerticalFlow);

                    std::cout << "Backward... " << std::flush;
                    flow(currentPointOfViewWithImage.second, lastPointOfViewWithImage.second, horizontalFlowGPU, verticalFlowGPU);
                    horizontalFlowGPU.download(backwardHorizontalFlow);
                    verticalFlowGPU.download(backwardVerticalFlow);
                }
                #endif

                std::cout << "Done." << std::endl;

                lastPointOfViewWithImage.second.release();

                std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
                std::vector<cv::Point2d> correctedLeftImagePoints, correctedRightImagePoints;

                scene->GetPointCloud()->ReserveSpace(image.cols * image.rows);
                leftImagePoints.reserve(image.cols * image.rows);
                rightImagePoints.reserve(image.cols * image.rows);

                const std::shared_ptr<Core::PointOfView> &leftPOV = lastPointOfViewWithImage.first;
                const std::shared_ptr<Core::PointOfView> &rightPOV = currentPointOfViewWithImage.first;
                std::vector<cv::Point2d> leftPoints, rightPoints;

                std::list<Core::Feature> commonPoints = scene->GetFeatures()->GetCommonPoints(leftPOV, rightPOV);
                leftPoints.reserve(commonPoints.size());
                rightPoints.reserve(commonPoints.size());

                for (const Core::Feature &point : commonPoints)
                {
                    Core::Projection leftProjection = point.GetProjection(leftPOV).get();
                    leftPoints.emplace_back(leftProjection.GetX(), leftProjection.GetY());

                    Core::Projection rightProjection = point.GetProjection(rightPOV).get();
                    rightPoints.emplace_back(rightProjection.GetX(), rightProjection.GetY());
                }

                cv::Mat fundamentalMatrix = cv::findFundamentalMat(leftPoints, rightPoints);

                int horizontalStart = 0.15 * image.cols;
                int horizontalEnd = 0.85 * image.cols;
                int verticalStart = 0.15 * image.rows;
                int verticalEnd = 0.85 * image.rows;

                #pragma omp parallel for
                for (int i = horizontalStart; i < horizontalEnd; i+=3)
                {
                    #pragma omp parallel for
                    for (int j = verticalStart; j < verticalEnd; j+=3)
                    {
                        double lowestError = 1e-0;

                        cv::Point2d left, right;

                        cv::Point2d leftPoint(i, j);
                        cv::Point2d rightPoint(
                                leftPoint.x + forwardHorizontalFlow.ptr<float>(static_cast<int>(leftPoint.y))[static_cast<int>(leftPoint.x)],
                                leftPoint.y + forwardVerticalFlow.ptr<float>(static_cast<int>(leftPoint.y))[static_cast<int>(leftPoint.x)]);
                        cv::Point2d leftPointBackwardOF(
                                rightPoint.x + backwardHorizontalFlow.ptr<float>(static_cast<int>(rightPoint.y))[static_cast<int>(rightPoint.x)],
                                rightPoint.y + backwardVerticalFlow.ptr<float>(static_cast<int>(rightPoint.y))[static_cast<int>(rightPoint.x)]);

                        cv::Point2d leftImagePoint(
                                    (leftPoint.x / scaleFactor),
                                    (leftPoint.y / scaleFactor));

                        cv::Point2d rightImagePoint(
                                    (rightPoint.x / scaleFactor),
                                    (rightPoint.y / scaleFactor));

                        cv::Point2d leftImagePointBackwardOF(
                                    (leftPointBackwardOF.x / scaleFactor),
                                    (leftPointBackwardOF.y / scaleFactor));

                        cv::Mat leftPointMat(3, 1, CV_64FC1), rightPointMat(3, 1, CV_64FC1), leftPointBackwardOFMat(3, 1, CV_64FC1);
                        leftPointMat.at<double>(0) = leftImagePoint.x;
                        leftPointMat.at<double>(1) = leftImagePoint.y;
                        leftPointMat.at<double>(2) = 1;

                        rightPointMat.at<double>(0) = rightImagePoint.x;
                        rightPointMat.at<double>(1) = rightImagePoint.y;
                        rightPointMat.at<double>(2) = 1;

                        leftPointBackwardOFMat.at<double>(0) = leftImagePointBackwardOF.x;
                        leftPointBackwardOFMat.at<double>(1) = leftImagePointBackwardOF.y;
                        leftPointBackwardOFMat.at<double>(2) = 1;

                        double error = std::fabs(cv::Mat(rightPointMat.t() * fundamentalMatrix * leftPointMat).at<double>(0, 0));
                        if (error < lowestError)
                        {
                            lowestError = error;

                            left = leftImagePoint;
                            right = rightImagePoint;
                        }
                        error = std::fabs(cv::Mat(rightPointMat.t() * fundamentalMatrix * leftPointBackwardOFMat).at<double>(0, 0));
                        if (error < lowestError)
                        {
                            lowestError = error;

                            left = leftImagePointBackwardOF;
                            right = rightImagePoint;
                        }

                        #pragma omp critical
//                        if (lowestError < 1e-0)
                        {
                            leftImagePoints.push_back(left);
                            rightImagePoints.push_back(right);
                        }
                    }
                }

                assert(leftImagePoints.size() == rightImagePoints.size());
                correctedLeftImagePoints.reserve(leftImagePoints.size());
                correctedRightImagePoints.reserve(rightImagePoints.size());
                cv::correctMatches(fundamentalMatrix, leftImagePoints, rightImagePoints, correctedLeftImagePoints, correctedRightImagePoints);
                assert(correctedLeftImagePoints.size() == correctedRightImagePoints.size());

                #pragma omp parallel for
                for (int i = 0; i < correctedLeftImagePoints.size(); i++)
                {
//                    const cv::Point2d &leftImagePoint = correctedLeftImagePoints.at(i);
//                    const cv::Point2d &rightImagePoint = correctedRightImagePoints.at(i);

                    const cv::Point2d &leftImagePoint = leftImagePoints.at(i);
                    const cv::Point2d &rightImagePoint = rightImagePoints.at(i);

                    Core::Point point;
                    Core::IImage::Pixel pixel = pointOfView->GetImage()->GetPixel(
                                                    leftImagePoint.x,
                                                    leftImagePoint.y);

                    point.SetColor(pixel.red, pixel.green, pixel.blue);

                    point.AddProjection(Core::Projection(leftImagePoint.x, leftImagePoint.y, leftPOV, true));
                    point.AddProjection(Core::Projection(rightImagePoint.x, rightImagePoint.y, rightPOV, true));

                    point.Triangulate(true, true);

                    #pragma omp critical
                    {
//                        std::cout << point.GetX() << " " << point.GetY() << " " << point.GetZ() << std::endl;
//                        std::cout << static_cast<int>(point.GetR()) << " " << static_cast<int>(point.GetG()) << " " << static_cast<int>(point.GetB()) << std::endl;
                        scene->GetPointCloud()->AddPoint(std::move(point));
                    }
                }

                lastPointOfViewWithImage = currentPointOfViewWithImage;
            }

//            std::vector<Core::Point> DenseMatcher::GeneratePointCloud(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV, const cv::Mat &fundamentalMatrix)
//            {
//                std::vector<Core::Point> pointCloud;
//                cv::Mat leftImage = leftPOV->GetImage()->ToOpenCVMat();
//                cv::Mat rightImage = rightPOV->GetImage()->ToOpenCVMat();
//                cv::Mat leftImageResized, rightImageResized;
//                cv::Mat gray;

//                std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
//                std::vector<cv::Point2d> correctedLeftImagePoints, correctedRightImagePoints;

//                cv::resize(leftImage, leftImageResized, cv::Size(), 0.5, 0.5);
//                leftImageResized.convertTo(leftImageResized, CV_32F, 1.0 / 255.0);
//                cv::cvtColor(leftImageResized, gray, cv::COLOR_BGR2GRAY);
//                cv::gpu::GpuMat leftImageGPU; leftImageGPU.upload(gray);

//                cv::resize(rightImage, rightImageResized, cv::Size(), 0.5, 0.5);
//                rightImageResized.convertTo(rightImageResized, CV_32F, 1.0 / 255.0);
//                cv::cvtColor(rightImageResized, gray, cv::COLOR_BGR2GRAY);
//                cv::gpu::GpuMat rightImageGPU; rightImageGPU.upload(gray);

//                // TODO fix parameters, possibly allow to change them
//                float scale = 0.8;
//                float alpha = 0.197;
//                float gamma = 50.0;
//                int inner = 10;
//                int outer = 77;
//                int solver = 10;
//                float timeStep = 0.1;

//                std::cout << "Estimating optical flow... " << std::flush;

//                cv::gpu::BroxOpticalFlow flow(alpha, gamma, scale, inner, outer, solver);
//                //    cv::gpu::FarnebackOpticalFlow flow;

//                cv::gpu::GpuMat dfu, dfv;
//                flow(leftImageGPU, rightImageGPU, dfu, dfv);

//                std::cout << "Done." << std::endl;

//                cv::Mat fu(dfu);
//                cv::Mat fv(dfv);

//                int horizontalStart = 0.15 * leftImageResized.cols;
//                int horizontalEnd = 0.85 * leftImageResized.cols;
//                int verticalStart = 0.15 * leftImageResized.rows;
//                int verticalEnd = 0.85 * leftImageResized.rows;

//                pointCloud.reserve(((horizontalEnd - horizontalStart) / 3) * ((verticalEnd - verticalStart) / 3));
//                leftImagePoints.reserve(((horizontalEnd - horizontalStart) / 3) * ((verticalEnd - verticalStart) / 3));
//                rightImagePoints.reserve(((horizontalEnd - horizontalStart) / 3) * ((verticalEnd - verticalStart) / 3));

//                #pragma omp parallel for
//                for (int i = horizontalStart; i < horizontalEnd; i+=3)
//                {
//                    #pragma omp parallel for
//                    for (int j = verticalStart; j < verticalEnd; j+=3)
//                    {
//                        cv::Point2d leftPoint(i, j);
//                        cv::Point2d rightPoint(leftPoint.x + fu.at<float>(leftPoint), leftPoint.y + fv.at<float>(leftPoint));

//                        cv::Point2d leftImagePoint((leftPoint.x * 2) - (0.5 * leftPOV->GetImage()->GetSize().width),
//                                                   (leftPoint.y * 2) - (0.5 * leftPOV->GetImage()->GetSize().height));
//                        cv::Point2d rightImagePoint((rightPoint.x * 2) - (0.5 * rightPOV->GetImage()->GetSize().width),
//                                                    (rightPoint.y * 2) - (0.5 * rightPOV->GetImage()->GetSize().height));

//                        #pragma omp critical
//                        {
//                            leftImagePoints.push_back(leftImagePoint);
//                            rightImagePoints.push_back(rightImagePoint);
//                        }

//                        //            Mat p1(3, 1, CV_64FC1), p2(3, 1, CV_64FC1);
//                        //            p1.at<double>(0) = leftImagePoint.x;
//                        //            p1.at<double>(1) = leftImagePoint.y;
//                        //            p1.at<double>(2) = 1;

//                        //            p2.at<double>(0) = rightImagePoint.x;
//                        //            p2.at<double>(1) = rightImagePoint.y;
//                        //            p2.at<double>(2) = 1;

//                        //            if (fabsf(Mat(p2.t() * fundamentalMatrix * p1).at<double>(0, 0)) <= 1e-0)
//                        //            {
//                        //                Point point;
//                        //                point.SetPosition(leftImagePoint.x, leftImagePoint.y, 1);
//                        //                cv::Vec3b bgrPixel = leftImage.at<cv::Vec3b>(Point2d(leftPoint.x * 2, leftPoint.y * 2));
//                        //                point.SetColor(bgrPixel[2], bgrPixel[1], bgrPixel[0]);
//                        //                point.AddCorrespondence(leftPOV, leftImagePoint);
//                        //                point.AddCorrespondence(rightPOV, rightImagePoint);

//                        //                point.Triangulate(true, true);

//                        //                #pragma omp critical
//                        //                {
//                        //                    pointCloud.push_back(std::move(point));
//                        //                }

//                        //            }
//                        //cout << "x^T*F*x = " << p2.t() * fundamentalMatrix * p1 << endl;
//                        //cout << "x^T*F*x = " << p1.t() * fundamentalMatrix * p2 << endl;

//                    }
//                }

//                assert(leftImagePoints.size() == rightImagePoints.size());
//                correctedLeftImagePoints.reserve(leftImagePoints.size());
//                correctedRightImagePoints.reserve(rightImagePoints.size());
//                cv::correctMatches(fundamentalMatrix, leftImagePoints, rightImagePoints, correctedLeftImagePoints, correctedRightImagePoints);
//                assert(correctedLeftImagePoints.size() == correctedRightImagePoints.size());

//                #pragma omp parallel for
//                for (int i = 0; i < correctedLeftImagePoints.size(); i++)
//                {
//                    cv::Point2d leftImagePoint = correctedLeftImagePoints.at(i);
//                    cv::Point2d rightImagePoint = correctedRightImagePoints.at(i);

//                    Core::Point point;
//                    point.SetPosition(leftImagePoint.x, leftImagePoint.y, 1);
//                    cv::Vec3b bgrPixel = leftImage.at<cv::Vec3b>(cv::Point2d(leftImagePoint.x + (0.5 * leftPOV->GetImage()->GetSize().width),
//                                                                             leftImagePoint.y + (0.5 * leftPOV->GetImage()->GetSize().height)));
//                    point.SetColor(bgrPixel[2], bgrPixel[1], bgrPixel[0]);
//                    point.AddCorrespondence(leftPOV, leftImagePoint);
//                    point.AddCorrespondence(rightPOV, rightImagePoint);

//                    point.Triangulate(true, true);

//                    #pragma omp critical
//                    {
//                        pointCloud.push_back(std::move(point));
//                    }
//                }

//                return pointCloud;
//            }
        }
    }
}
