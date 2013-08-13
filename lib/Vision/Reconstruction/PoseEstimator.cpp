#include "PoseEstimator.h"

#include <eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "Vision/Core/Feature.h"
#include "Vision/Core/FeatureSet.h"
#include "Vision/Core/PointOfView.h"
#include "Vision/Core/Scene.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            PoseEstimator::PoseEstimator(const std::shared_ptr<Core::Scene> &scene)
                : scene(scene)
            {
            }

            void PoseEstimator::EstimateCameraPose(std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                // TODO implement RANSAC

                std::vector<cv::Point3f> pointCloud;
                std::vector<cv::Point2f> imagePoints;

                for (auto it = scene->GetFeatures()->Begin(); it != scene->GetFeatures()->End(); it++)
                {
                    Core::Feature &feature = *it;
                    if (feature.IsTriangulated())
                    {
                        boost::optional<Core::Projection> imagePoint = feature.GetProjection(pointOfView);

                        if (imagePoint.is_initialized())
                        {
                            pointCloud.push_back(cv::Point3f(static_cast<float>(feature.GetX()), static_cast<float>(feature.GetY()), static_cast<float>(feature.GetZ())));
                            imagePoints.push_back(cv::Point2f(static_cast<float>(imagePoint->GetX()), static_cast<float>(imagePoint->GetY())));
                        }
                    }
                }

                assert(pointCloud.size() == imagePoints.size());
                int pointCount = pointCloud.size();

                if (pointCount < 9)
                {
                    std::cout << "Not enough points to find pose: " << pointCloud.size() << std::endl;
                    return;
                }

                Eigen::MatrixXd A(2 * pointCount, 11);
                Eigen::MatrixXd B(2 * pointCount, 1);

                for (int i = 0; i < pointCount; i++)
                {
                    A(2 * i + 0, 0) = pointCloud.at(i).x;
                    A(2 * i + 0, 1) = pointCloud.at(i).y;
                    A(2 * i + 0, 2) = pointCloud.at(i).z;
                    A(2 * i + 0, 3) = 1.0;

                    A(2 * i + 0, 4) = 0;
                    A(2 * i + 0, 5) = 0;
                    A(2 * i + 0, 6) = 0;
                    A(2 * i + 0, 7) = 0;

                    A(2 * i + 0, 8) = imagePoints.at(i).x * pointCloud.at(i).x;
                    A(2 * i + 0, 9) = imagePoints.at(i).x * pointCloud.at(i).y;
                    A(2 * i + 0, 10) = imagePoints.at(i).x * pointCloud.at(i).z;

                    B(2 * i + 0, 0) = -imagePoints.at(i).x;

                    A(2 * i + 1, 0) = 0;
                    A(2 * i + 1, 1) = 0;
                    A(2 * i + 1, 2) = 0;
                    A(2 * i + 1, 3) = 0;

                    A(2 * i + 1, 4) = pointCloud.at(i).x;
                    A(2 * i + 1, 5) = pointCloud.at(i).y;
                    A(2 * i + 1, 6) = pointCloud.at(i).z;
                    A(2 * i + 1, 7) = 1.0;

                    A(2 * i + 1, 8) = imagePoints.at(i).y * pointCloud.at(i).x;
                    A(2 * i + 1, 9) = imagePoints.at(i).y * pointCloud.at(i).y;
                    A(2 * i + 1, 10) = imagePoints.at(i).y * pointCloud.at(i).z;

                    B(2 * i + 1, 0) = -imagePoints.at(i).y;
                }

                Eigen::MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
//                Eigen::MatrixXd X = A.householderQr().solve(B);

                Eigen::MatrixXd P(3, 4);
                P <<    X(0), X(1), X(2), X(3),
                        X(4), X(5), X(6), X(7),
                        X(8), X(9), X(10), 1.0;

                double error = 0.0;

                for (int i = 0; i < pointCount; i++)
                {
                    Eigen::MatrixXd point(4, 1);
                    point << pointCloud.at(i).x,
                            pointCloud.at(i).y,
                            pointCloud.at(i).z,
                            1.0;

                    Eigen::MatrixXd projectedPoint = P * point;
                    projectedPoint(0, 0) /= -projectedPoint(2, 0);
                    projectedPoint(1, 0) /= -projectedPoint(2, 0);

                    double dx = projectedPoint(0, 0) - imagePoints.at(i).x;
                    double dy = projectedPoint(1, 0) - imagePoints.at(i).y;

                    double distance = dx * dx + dy * dy;

                    error += distance;
                }

                std::cout << "Mean reprojection error: " << error / static_cast<double>(pointCount) << std::endl;

                cv::Mat poseMatrix; cv::eigen2cv(P, poseMatrix);
                pointOfView->GetCameraParameters().SetPoseMatrix(poseMatrix);
                pointOfView->GetCameraParameters().SetPoseDetermined(false);
            }
        }
    }
}
