#include "PoseEstimator.h"

#include <functional>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include <opencv2/core/eigen.hpp>

#include "Math/Statistics/RANSAC.h"

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
                  threshold(4.0)
            {
            }

            void PoseEstimator::EstimateCameraPose(std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                using namespace std::placeholders;
                Math::Statistics::RANSAC<DataType, ParametersType> ransacEngine(std::bind(&PoseEstimator::EstimateProjection, this, _1),
                                                                                std::bind(&PoseEstimator::EstimateError, this, _1, _2, false, 1),
                                                                                6, threshold, 2048);

                std::vector<DataType> data;
                data.reserve(scene->GetFeatures()->Size());
                for (auto it = scene->GetFeatures()->Begin(); it != scene->GetFeatures()->End(); ++it)
                {
                    Core::Feature &feature = *it;
                    if (feature.IsTriangulated())
                    {
                        boost::optional<Core::Projection> projection = feature.GetProjection(pointOfView);

                        if (projection.is_initialized())
                        {
                            data.push_back(std::make_pair(feature, projection.get()));
                        }
                    }
                }

                if (data.size() < 6)
                {
                    std::cout << "Not enough points to estimate camera pose." << std::endl;
                    return;
                }

                Math::LinearAlgebra::Matrix<3, 4> projectionMatrix = ransacEngine.Estimate(data);

                auto qr = projectionMatrix.block(0, 0, 3, 3).colwise().reverse().transpose().householderQr();
                Eigen::MatrixXd tempQ = qr.householderQ();
                Eigen::MatrixXd tempR = qr.matrixQR().triangularView<Eigen::Upper>();

                Eigen::MatrixXd q = tempQ.transpose().colwise().reverse();
                Eigen::MatrixXd r = tempR.transpose().colwise().reverse().rowwise().reverse();

                int negative = static_cast<int>(r(0) < 0) + static_cast<int>(r(4) < 0) + static_cast<int>(r(8) < 0);
                int sign = (negative % 2 == 0) ? 1 : -1;

                std::vector<DataType> inliers;
                for (const DataType &datum : data)
                {
                    Math::Core::Number error = EstimateError(projectionMatrix, datum, true, sign);

                    if (error < threshold * threshold)
                    {
                        inliers.push_back(datum);
                    }
                }

                if (inliers.size() < 6)
                {
                    std::cout << "Not enough inliers to estimate camera pose." << std::endl;
                    return;
                }

//                ParametersType projectionMatrixLinear = EstimateProjection(inliers);

//                for (const DataType &datum : data)
//                {
//                    Math::Core::Number error = EstimateError(projectionMatrixLinear, datum, true, sign);

//                    if (error < threshold * threshold)
//                    {
//                        inliers.push_back(datum);
//                    }
//                }


//                std::cout << "Intrinsics: " << std::endl << r << std::endl;
//                std::cout << "Extrinsics: " << std::endl << q << std::endl;

//                cv::Mat poseMatrix; cv::eigen2cv(q, poseMatrix);
//                pointOfView->GetCameraParameters().SetPoseMatrix(poseMatrix);
//                pointOfView->GetCameraParameters().SetPoseDetermined(false);
            }

            PoseEstimator::ParametersType PoseEstimator::EstimateProjection(const std::vector<DataType> &data)
            {
                Eigen::MatrixXd A(2 * data.size(), 11);
                Eigen::MatrixXd B(2 * data.size(), 1);

                for (int i = 0; i < data.size(); i++)
                {
                    A(2 * i + 0, 0) = data.at(i).first.GetX();
                    A(2 * i + 0, 1) = data.at(i).first.GetY();
                    A(2 * i + 0, 2) = data.at(i).first.GetZ();
                    A(2 * i + 0, 3) = 1.0;

                    A(2 * i + 0, 4) = 0;
                    A(2 * i + 0, 5) = 0;
                    A(2 * i + 0, 6) = 0;
                    A(2 * i + 0, 7) = 0;

                    A(2 * i + 0, 8) = data.at(i).second.GetX() * data.at(i).first.GetX();
                    A(2 * i + 0, 9) = data.at(i).second.GetX() * data.at(i).first.GetY();
                    A(2 * i + 0, 10) = data.at(i).second.GetX() * data.at(i).first.GetZ();

                    B(2 * i + 0, 0) = -data.at(i).second.GetX();

                    A(2 * i + 1, 0) = 0;
                    A(2 * i + 1, 1) = 0;
                    A(2 * i + 1, 2) = 0;
                    A(2 * i + 1, 3) = 0;

                    A(2 * i + 1, 4) = data.at(i).first.GetX();
                    A(2 * i + 1, 5) = data.at(i).first.GetY();
                    A(2 * i + 1, 6) = data.at(i).first.GetZ();
                    A(2 * i + 1, 7) = 1.0;

                    A(2 * i + 1, 8) = data.at(i).second.GetY() * data.at(i).first.GetX();
                    A(2 * i + 1, 9) = data.at(i).second.GetY() * data.at(i).first.GetY();
                    A(2 * i + 1, 10) = data.at(i).second.GetY() * data.at(i).first.GetZ();

                    B(2 * i + 1, 0) = -data.at(i).second.GetY();
                }

//                Eigen::MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
                Eigen::MatrixXd X = A.householderQr().solve(B);

                Math::LinearAlgebra::Matrix<3, 4> projectionMatrix;
                projectionMatrix <<
                        X(0), X(1), X(2), X(3),
                        X(4), X(5), X(6), X(7),
                        X(8), X(9), X(10), 1.0;

                return projectionMatrix;
            }

            Math::Core::Number PoseEstimator::EstimateError(const ParametersType &projectionMatrix, const DataType &datum, bool checkCheirality, int sign)
            {
                Eigen::MatrixXd point(4, 1);
                point << datum.first.GetX(),
                        datum.first.GetY(),
                        datum.first.GetZ(),
                        1.0;

                Eigen::MatrixXd projectedPoint = projectionMatrix * point;
                if (checkCheirality && sign * projectedPoint(2) > 0)
                {
                    return std::numeric_limits<double>::max();
                }

                projectedPoint(0) /= -projectedPoint(2);
                projectedPoint(1) /= -projectedPoint(2);

                double dx = projectedPoint(0) - datum.second.GetX();
                double dy = projectedPoint(1) - datum.second.GetY();

                double distance = dx * dx + dy * dy;

                return distance;
            }
        }
    }
}
