#include "PoseEstimator.h"

#include <functional>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include <opencv2/core/eigen.hpp>

#include "Math/Statistics/RANSAC.h"
#include "Math/Optimization/LevenbergMarquardt.h"

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
                : scene(scene),
                  threshold(8.0)
            {
            }

            void PoseEstimator::EstimateCameraPose(std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                std::vector<Core::Feature *> features;
                std::vector<cv::Point3f> pointCloud;
                std::vector<cv::Point2f> imagePoints;
                for (auto it = scene->GetFeatures()->Begin(); it != scene->GetFeatures()->End(); ++it)
                {
                    Core::Feature &feature = *it;
                    if (feature.IsTriangulated())
                    {
                        boost::optional<Core::Projection> projection = feature.GetProjection(pointOfView);

                        if (projection.is_initialized())
                        {
                            features.push_back(&feature);
                            pointCloud.emplace_back(feature.GetX(), feature.GetY(), feature.GetZ());
                            imagePoints.emplace_back(projection->GetX(), projection->GetY());
                        }
                    }
                }

                assert (pointCloud.size() == imagePoints.size());
                if (pointCloud.size() < 6)
                {
                    std::cout << "Not enough points to estimate camera pose." << std::endl;
                    return;
                }

                cv::Mat cameraMatrix = pointOfView->GetCameraParameters().GetCameraMatrix();
                cv::Mat distortionCoefficients = pointOfView->GetCameraParameters().GetDistortionCoefficients();

                cv::Mat rotation, translation;
                std::vector<int> inliers;
                cv::solvePnPRansac(pointCloud, imagePoints, cameraMatrix, distortionCoefficients, rotation, translation, false, 100, threshold, 0.9 * pointCloud.size(), inliers);

                // optimize for R, T and f

                auto cameraRefineFunctor = [&inliers, &pointCloud, &imagePoints](const Eigen::VectorXd &input) -> Eigen::VectorXd
                {
                    Math::LinearAlgebra::Matrix<3, 3> K = Eigen::Matrix3d::Identity();
                    K(0, 0) = K(1, 1) = input(6);
                    Math::LinearAlgebra::Matrix<3, 4> P;

                    cv::Mat rvec(3, 1, CV_64FC1), rmat;
                    rvec.at<double>(0) = input(0);
                    rvec.at<double>(1) = input(1);
                    rvec.at<double>(2) = input(2);
                    Eigen::Matrix3d rotationMatrix;
                    cv::Rodrigues(rvec, rmat);
                    cv::cv2eigen(rmat, rotationMatrix);

                    Eigen::Vector3d translationVector;
                    translationVector << input(3), input(4), input(5);

                    P.block(0, 0, 3, 3) = rotationMatrix;
                    P.col(3) = translationVector;

                    Math::LinearAlgebra::Matrix<3, 4> KP = K * P;

                    Eigen::VectorXd results(2 * inliers.size());

                    #pragma omp parallel for
                    for (std::size_t i = 0; i < inliers.size(); i++)
                    {
                        cv::Point3f objectPoint = pointCloud.at(inliers.at(i));
                        cv::Point2f imagePoint = imagePoints.at(inliers.at(i));

                        Eigen::Vector4d point;
                        point << objectPoint.x,
                                objectPoint.y,
                                objectPoint.z,
                                1;

                        Eigen::Vector3d projectedPoint = KP * point;
                        projectedPoint.head<2>() /= projectedPoint(2);

                        results(2 * i + 0) = std::abs(imagePoint.x - projectedPoint(0));
                        results(2 * i + 1) = std::abs(imagePoint.y - projectedPoint(1));
                    }

                    return results;
                };

                Eigen::Vector3d rotationVector, translationVector;
                cv::cv2eigen(rotation, rotationVector);
                cv::cv2eigen(translation, translationVector);

                Eigen::VectorXd projection(7);
                double focalLength = cameraMatrix.at<double>(0, 0);
                projection << rotationVector, translationVector, focalLength;
                bool more = false;
                do
                {
                    Math::Optimization::LevenbergMarquardt<> lm(cameraRefineFunctor, 7, 2 * inliers.size());
                    lm.Minimize(projection);

                    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
                    Eigen::Matrix3d rotationMatrix;
                    focalLength = projection(6);
                    K(0, 0) = K(1, 1) = focalLength;
                    rotationVector << projection(0), projection(1), projection(2);
                    cv::Mat srcR, dstR;
                    cv::eigen2cv(rotationVector, srcR);
                    cv::Rodrigues(srcR, dstR);
                    cv::cv2eigen(dstR, rotationMatrix);
                    translationVector << projection(3), projection(4), projection(5);
                    Math::LinearAlgebra::Matrix<3, 4> poseMatrix;
                    poseMatrix << rotationMatrix, translationVector;
                    Math::LinearAlgebra::Matrix<3, 4> projectionMatrix = K * poseMatrix;

                    more = false;

                    inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&more, &projectionMatrix, &features, &pointOfView, &pointCloud, &imagePoints, this](int index)
                    {
                        cv::Point3f objectPoint = pointCloud.at(index);
                        cv::Point2f imagePoint = imagePoints.at(index);

                        Eigen::Vector4d point;
                        point << objectPoint.x,
                                objectPoint.y,
                                objectPoint.z,
                                1;

                        Eigen::Vector3d projectedPoint = projectionMatrix * point;
                        projectedPoint.head<2>() /= projectedPoint(2);

                        double dx = std::abs(imagePoint.x - projectedPoint(0));
                        double dy = std::abs(imagePoint.y - projectedPoint(1));

                        double error = std::sqrt(dx * dx + dy * dy);

                        if (error > threshold * threshold)
                        {
                            features.at(index)->RemoveProjection(pointOfView);
                            more = true;
                            return true;
                        }
                        return false;
                    }), inliers.end());
                }
                while (more);

                focalLength = projection(6);
                cv::Mat rotationMatrix, translationMatrix;

                cv::Mat rotationVec;
                cv::eigen2cv(rotationVector, rotationVec);
                cv::Rodrigues(rotationVec, rotationMatrix);

                cv::eigen2cv(translationVector, translationMatrix);

                pointOfView->GetCameraParameters().SetRotationMatrix(rotationMatrix);
                pointOfView->GetCameraParameters().SetTranslationMatrix(-1.0 * (rotationMatrix.t() * translationMatrix));
                pointOfView->GetCameraParameters().SetFocalLength(focalLength);

                pointOfView->GetCameraParameters().SetPoseDetermined(true);
            }
        }
    }
}
