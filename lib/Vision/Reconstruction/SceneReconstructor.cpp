#include "Vision/Reconstruction/SceneReconstructor.h"

#include <algorithm>
#include <math.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <five-point.hpp>

#include "Vision/Core/Feature.h"
#include "Vision/Core/FeatureSet.h"
#include "Vision/Core/PointOfView.h"
#include "Vision/Core/PointCloud.h"
#include "Vision/Core/Scene.h"
#include "Vision/Core/SingleViewCamera.h"
#include "Vision/Reconstruction/BundleAdjuster.h"
#include "Vision/Reconstruction/DenseMatcher.h"
#include "Vision/Reconstruction/SURFGPUFeatureMatcher.h"
#include "Vision/Reconstruction/GFTTFeatureMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            SceneReconstructor::SceneReconstructor(std::shared_ptr<Core::SingleViewCamera> camera)
                : camera(camera),
                  scene(new Core::Scene()),
                  featureMatcher(new SURFGPUFeatureMatcher(*scene)),
//                  featureMatcher(new GFTTFeatureMatcher(*scene)),
                  bundleAdjuster(new BundleAdjuster(*scene)),
                  denseMatcher(new DenseMatcher(scene)),
                  running(false),
                  initialTriangulation(false)
            {
            }

            SceneReconstructor::~SceneReconstructor()
            {
            }

            bool SceneReconstructor::IsInitialTriangulationReady() const
            {
                return initialTriangulation;
            }

            std::shared_ptr<Core::Scene> SceneReconstructor::GetScene() const
            {
                return scene;
            }

            void SceneReconstructor::Run()
            {
                running = true;
                while (running && camera->HasNextPointOfView())
                {
                    std::shared_ptr<Core::PointOfView> pointOfView = camera->GetNextPointOfView();
                    featureMatcher->ProcessImage(pointOfView);

                    AddForReconstruction(pointOfView);
                }
            }

            void SceneReconstructor::TriangulatePoints(bool optimize)
            {
                std::cout << "Triangulating points" << std::endl;

//                #pragma omp parallel for
                for (auto it = scene->GetFeatures()->Begin(); it != scene->GetFeatures()->End(); it++)
                {
                    Core::Feature &feature = *it;
                    feature.Triangulate(!initialTriangulation, optimize);
                }
            }

            void SceneReconstructor::InitialReconstruction()
            {
                double focalLength = currentPointOfView->GetCameraParameters().GetFocalLength();

                // pair<pov*, pov*>
                auto mostMatchesPair = scene->GetFeatures()->FindPairWithMostMatches();
                std::shared_ptr<Core::PointOfView> leftPOV = mostMatchesPair.first;
                std::shared_ptr<Core::PointOfView> rightPOV = mostMatchesPair.second;

                std::list<Core::Feature> commonFeatures = scene->GetFeatures()->GetCommonPoints(leftPOV, rightPOV);

                std::vector<cv::Point2d> leftImagePoints, rightImagePoints;

                leftImagePoints.reserve(commonFeatures.size());
                rightImagePoints.reserve(commonFeatures.size());
                for (const Core::Feature &feature : commonFeatures)
                {
                    const Core::Projection &leftProjection = feature.GetProjection(leftPOV).get();
                    leftImagePoints.push_back(cv::Point2d(leftProjection.GetX(), leftProjection.GetY()));
                    const Core::Projection &rightProjection = feature.GetProjection(rightPOV).get();
                    rightImagePoints.push_back(cv::Point2d(rightProjection.GetX(), rightProjection.GetY()));
                }

                if (leftImagePoints.size() == 0 || rightImagePoints.size() == 0)
                {
                    return;
                }

                cv::Mat essentialMatrix = findEssentialMat(leftImagePoints, rightImagePoints, focalLength);
                essentialMatrix.at<double>(0, 2) = -essentialMatrix.at<double>(0, 2);
                essentialMatrix.at<double>(1, 2) = -essentialMatrix.at<double>(1, 2);
                essentialMatrix.at<double>(2, 0) = -essentialMatrix.at<double>(2, 0);
                essentialMatrix.at<double>(2, 1) = -essentialMatrix.at<double>(2, 1);

                cv::Mat rotationMatrix, translationMatrix;
                recoverPose(essentialMatrix, leftImagePoints, rightImagePoints, rotationMatrix, translationMatrix, focalLength);

                leftPOV->GetCameraParameters().SetRotationMatrix(cv::Mat::eye(3, 3, CV_64FC1));
                leftPOV->GetCameraParameters().SetTranslationMatrix(cv::Mat::zeros(3, 1, CV_64FC1));

                rightPOV->GetCameraParameters().SetRotationMatrix(rotationMatrix);
                rightPOV->GetCameraParameters().SetTranslationMatrix(-1.0 * (rotationMatrix.t() * translationMatrix));

                leftPOV->GetCameraParameters().SetPoseDetermined(true);
                rightPOV->GetCameraParameters().SetPoseDetermined(true);

                TriangulatePoints(true);
                initialTriangulation = true;

                if (!initialTriangulation)
                {
                    leftPOV->GetCameraParameters().SetPoseDetermined(false);
                    rightPOV->GetCameraParameters().SetPoseDetermined(false);
                }

            }

            bool SceneReconstructor::EstimateCameraPose(const std::shared_ptr<Core::PointOfView> &pointOfView, cv::Mat &rotationMatrix, cv::Mat &translationMatrix)
            {
                // TODO implement RANSAC

                std::vector<cv::Point3f> pointCloud;
                std::vector<cv::Point2f> imagePoints;

                for (int i = 0; i < features->Size(); i++)
                {
                    std::shared_ptr<Core::Feature> feature = features->GetFeature(i);
                    if (feature->IsTriangulated() && !feature->IsHidden())
                    {
                        if (feature->HasCorrespondenceInView(pointOfView))
                        {
                            cv::Point3d cloudPoint = feature->GetPoint3d();
                            cv::Point2d imagePoint = feature->GetPointInView(pointOfView);

                            pointCloud.push_back(cv::Point3f(static_cast<float>(cloudPoint.x), static_cast<float>(cloudPoint.y), static_cast<float>(cloudPoint.z)));
                            imagePoints.push_back(cv::Point2f(static_cast<float>(imagePoint.x), static_cast<float>(imagePoint.y)));
                        }
                    }
                }

                assert(pointCloud.size() == imagePoints.size());
                int pointCount = pointCloud.size();

                if (pointCount)
                {
                    std::cout << "Not enough points to find pose: " << pointCloud.size() << std::endl;
                    return false;
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
                    A(2 * i + 0, 9) = imagePoints.at(i).y * pointCloud.at(i).y;
                    A(2 * i + 0, 10) = imagePoints.at(i).z * pointCloud.at(i).z;

                    B(2 * i + 0, 0) = -imagePoints.at(i).x;

                    A(2 * i + 1, 0) = 0;
                    A(2 * i + 1, 1) = 0;
                    A(2 * i + 1, 2) = 0;
                    A(2 * i + 1, 3) = 0;

                    A(2 * i + 1, 4) = pointCloud.at(i).x;
                    A(2 * i + 1, 5) = pointCloud.at(i).y;
                    A(2 * i + 1, 6) = pointCloud.at(i).z;
                    A(2 * i + 1, 7) = 1.0;

                    A(2 * i + 1, 8) = imagePoints.at(i).x * pointCloud.at(i).x;
                    A(2 * i + 1, 9) = imagePoints.at(i).y * pointCloud.at(i).y;
                    A(2 * i + 1, 10) = imagePoints.at(i).z * pointCloud.at(i).z;

                    B(2 * i + 1, 0) = -imagePoints.at(i).x;
                }

                Eigen::MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

                double error = 0.0;

                for (int i = 0; i < pointCount; i++)
                {
                    Eigen::MatrixXd point(4, 1);
                    point << pointCloud.at(i).x,
                            pointCloud.at(i).y,
                            pointCloud.at(i).z,
                            1.0;

                    Eigen::MatrixXd projectedPoint = X * point;
                    projectedPoint(0, 0) /= -projectedPoint(2, 0);
                    projectedPoint(1, 0) /= -projectedPoint(2, 0);

                    double dx = projectedPoint(0, 0) - imagePoints.at(i).x;
                    double dy = projectedPoint(1, 0) - imagePoints.at(i).y;

                    double distance = dx * dx + dy * dy;

                    error += distance;
                }

                std::cout << "Mean reprojection error: " << error << std::endl;

                // TODO finish

                /*
                cv::Mat cameraMatrix = pointOfView->GetCameraParameters().GetCameraMatrix();
                cv::Mat distortionCoefficients = pointOfView->GetCameraParameters().GetDistortionCoefficients();
                //    Mat cameraMatrix32f, distortionCoefficients32f;
                //    cameraMatrix.convertTo(cameraMatrix32f, CV_32FC1);
                //    distortionCoefficients.convertTo(distortionCoefficients32f, CV_32FC1);

                //    cv::Mat pointCloudMat(pointCloud); pointCloudMat = pointCloudMat.t();
                //    cv::Mat imagePointsMat(imagePoints); imagePointsMat = imagePointsMat.t();
                //    cv::Mat rotationVector, translationVector;

                //    cv::gpu::solvePnPRansac(pointCloudMat, imagePointsMat, cameraMatrix32f, distortionCoefficients32f, rotationVector, translationVector, false);

                cv::Mat prevPOVRot = lastPointOfView->GetCameraParameters().GetRotationMatrix();
                cv::Mat rotation; cv::Rodrigues(prevPOVRot, rotation);
                cv::Mat translation = lastPointOfView->GetCameraParameters().GetTranslationMatrix();
                //    rotationVector.convertTo(rotation, CV_64FC1);
                //    translationVector.convertTo(translation, CV_64FC1);

                cv::solvePnPRansac(pointCloud, imagePoints, cameraMatrix, distortionCoefficients, rotation, translation, true);
//                cv::solvePnP(pointCloud, imagePoints, cameraMatrix, distortionCoefficients, rotation, translation);

                std::vector<cv::Point2f> projectedPoints;
                cv::projectPoints(pointCloud, rotation, translation, cameraMatrix, distortionCoefficients, projectedPoints);

                std::vector<int> inliers;
                std::vector<double> reprojectionErrors;
                for (int i = 0; i < projectedPoints.size(); i++)
                {
                    reprojectionErrors.push_back(cv::norm(projectedPoints[i] - imagePoints[i]));
                    if (cv::norm(projectedPoints[i] - imagePoints[i]) < 10.0)
                    {
                        inliers.push_back(i);
                    }
                }

                if (inliers.size() < (double)(imagePoints.size()) /  5.0)
                {
                    rotationMatrix = cv::Mat();
                    translationMatrix = cv::Mat();
                    std::cout << "Not enough inliers: " << inliers.size() << " / " << imagePoints.size() << std::endl;
                    return false;
                }

                cv::Rodrigues(rotation, rotationMatrix);

                std::cout << "SolvePnP output:" << std::endl;
                std::cout << rotation << std::endl;
                std::cout << rotationMatrix << std::endl;
                std::cout << translation << std::endl;

                translationMatrix = (-1.0 * rotationMatrix.t()) * translation;
                rotationMatrix.at<double>(2, 0) *= -1;
                rotationMatrix.at<double>(2, 1) *= -1;
                rotationMatrix.at<double>(2, 2) *= -1;
                rotationMatrix *= -1;
                if (cv::norm(translationMatrix) > 200.0)
                {
                    std::cout << cv::norm(translationMatrix) << std::endl;
                    rotationMatrix = cv::Mat();
                    translationMatrix = cv::Mat();
                    std::cout << "Estimated camera movement too big." << std::endl;
                    return false;
                }

                if (cv::determinant(rotationMatrix) + 1.0 < 1e-09)
                {
                    rotationMatrix = cv::Mat();
                    translationMatrix = cv::Mat();
                    std::cout << "Rotation incoherent." << std::endl;
                    return false;
                }


                std::cout << "Pose estimation successful with " << inliers.size() << " points." << std::endl;
                */

                return true;
            }


            void SceneReconstructor::AddForReconstruction(std::shared_ptr<Core::PointOfView> pointOfView)
            {
                if (!pointOfView->GetImage()->HasDepth())
                {
                    if (lastPointOfView == NULL)
                    {
                        scene->ResetFeatures();
                        bundleAdjuster->Reset(*scene);

                        initialTriangulation = false;
                        lastPointOfView = pointOfView;
                        return;
                    }

                    if (lastPointOfView == pointOfView)
                    {
                        return;
                    }

                    currentPointOfView = pointOfView;

                    if (!initialTriangulation)
                    {
                        InitialReconstruction();

                        lastPointOfView = currentPointOfView;

                        return;
                    }

                    cv::Mat rotationMatrix, translationMatrix;
                    if (!EstimateCameraPose(currentPointOfView, rotationMatrix, translationMatrix))
                    {
                        std::cout << "Failed to estimate camera pose." << std::endl;

                        lastPointOfView = currentPointOfView;

                        return;
                    }

                    currentPointOfView->GetCameraParameters().SetPoseDetermined(false);

//                    shouldRunBundleAdjustment = true;
//                    bundleAdjuster->RunOnAllData();

                    lastPointOfView = currentPointOfView;
                }
                else
                {
                    throw std::runtime_error("Reconstruction from depth maps is not yet implemented.");
                }
            }
        }
    }
}
