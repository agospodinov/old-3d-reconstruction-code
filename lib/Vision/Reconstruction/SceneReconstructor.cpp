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
        namespace Reconstruction {

            SceneReconstructor::SceneReconstructor(std::shared_ptr<Core::SingleViewCamera> camera)
                : camera(camera),
                  featureMatcher(new SURFGPUFeatureMatcher()),
//                  featureMatcher(new GFTTFeatureMatcher()),
                  bundleAdjuster(new BundleAdjuster()),
                  scene(new Core::Scene()),
                  currentPointOfView(NULL),
                  lastPointOfView(NULL),
                  running(false),
                  initialTriangulation(false),
                  shouldRunBundleAdjustment(false)
            {
                denseMatcher = std::unique_ptr<DenseMatcher>(new DenseMatcher(scene));
            }

            SceneReconstructor::~SceneReconstructor()
            {
                running = false;

                if (featureMatchingThread.joinable())
                {
                    featureMatchingThread.join();
                }

                if (reconstructionThread.joinable())
                {
                    reconstructionThread.join();
                }

                if(bundleAdjustmentThread.joinable())
                {
                    bundleAdjustmentThread.join();
                }
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
                running = false;

                if (featureMatchingThread.joinable())
                {
                    featureMatchingThread.join();
                }

                if (reconstructionThread.joinable())
                {
                    reconstructionThread.join();
                }

                if(bundleAdjustmentThread.joinable())
                {
                    bundleAdjustmentThread.join();
                }

                running = true;

                featureMatchingThread = std::thread(std::bind(&SceneReconstructor::RunFeatureMatching, this));
                reconstructionThread = std::thread(std::bind(&SceneReconstructor::RunReconstruction, this));
                bundleAdjustmentThread = std::thread(std::bind(&SceneReconstructor::RunBundleAdjustment, this));
            }

            void SceneReconstructor::RunFeatureMatching()
            {
                while (running && camera->HasNextPointOfView())
                {
                    std::shared_ptr<Core::PointOfView> pointOfView = camera->GetNextPointOfView();
                    featureMatcher->AddImage(pointOfView);
                    std::lock_guard<std::mutex> lock(pointOfViewQueueMutex);
                    matchedPointsOfView.push(pointOfView);
                }
            }

            void SceneReconstructor::RunReconstruction()
            {
                while (running)
                {
                    if (!matchedPointsOfView.empty())
                    {
                        std::shared_ptr<Core::PointOfView> pointOfView;
                        {
                            std::lock_guard<std::mutex> lock(pointOfViewQueueMutex);
                            pointOfView = matchedPointsOfView.front();
                            matchedPointsOfView.pop();
                        }

                        AddForReconstruction(pointOfView);
                    }
                }
            }

            void SceneReconstructor::RunBundleAdjustment()
            {
                while (running)
                {
                    if (shouldRunBundleAdjustment)
                    {
                        bundleAdjuster->RunOnNewData();
                        scene->UpdatePoints();
                        shouldRunBundleAdjustment = false;
                    }
                }
            }

            void SceneReconstructor::TriangulatePoints(bool optimize)
            {
                std::cout << "Triangulating points" << std::endl;

                #pragma omp parallel for
                for (int i = 0; i < features->Size(); i++)
                {
                    std::shared_ptr<Core::Feature> feature = features->GetFeature(i);
                    feature->Triangulate(!initialTriangulation, optimize);
                }
            }

            void SceneReconstructor::RejectBadPoints(const std::shared_ptr<Core::PointOfView> &pointOfView, const std::vector<int> &featureIndices, const std::vector<uchar> &status)
            {
                std::cout << "Getting rid of bad points." << std::endl;
                int badPointsCount = 0;
                for (int i = 0; i < status.size(); i++)
                {
                    int featureIndex = featureIndices[i];
                    std::shared_ptr<Core::Feature> feature = features->GetFeature(featureIndex);

                    if (feature->IsTriangulated())
                    {
                        if (status[i] == 0)
                        {
                            // Feature was not in front of camera
                            feature->SetHidden(true);
                            badPointsCount++;
                            continue;
                        }

                        double reprojectionError = feature->EstimateError(pointOfView);
                        if (reprojectionError > 6.0)
                        {
//                            std::cout << reprojectionError << std::endl;
                            feature->SetHidden(true);
                            badPointsCount++;
                            continue;
                        }
                    }
                }
                std::cout << "Removed " << badPointsCount << " points." << std::endl;
            }

            void SceneReconstructor::InitialReconstruction()
            {
                // TODO These two for loops, are they logically correct?
                for (int i = 0; i < features->Size(); i++)
                {
                    std::shared_ptr<Core::Feature> feature = features->GetFeature(i);
                    feature->SetTriangulated(false);
                }

                for (std::shared_ptr<Core::PointOfView> pointOfView : features->GetPointsOfView())
                {
                    pointOfView->GetCameraParameters().SetPoseDetermined(false);
                }

                double focalLength = currentPointOfView->GetCameraParameters().GetFocalLength();

                // pair<pov*, pov*>
                auto mostMatchesPair = features->FindPairWithMostMatches();
                std::shared_ptr<Core::PointOfView> leftPOV = mostMatchesPair.first;
                std::shared_ptr<Core::PointOfView> rightPOV = mostMatchesPair.second;

                std::vector<cv::Point2d> leftImagePoints, rightImagePoints;
                features->GetCommonPoints(leftPOV, rightPOV, leftImagePoints, rightImagePoints);

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

                if (initialTriangulation)
                {
                    shouldRunBundleAdjustment = true;
                    bundleAdjuster->RunOnAllData();
                    denseMatcher->Add(leftPOV);
                    denseMatcher->Add(rightPOV);

//                    for (int i = 0; i < scene->GetFeatures()->Size(); i++)
//                    {
//                        const std::shared_ptr<Core::Feature> &feature = scene->GetFeatures()->GetFeature(i);
//                        scene->GetPointCloud()->AddPoint(*(feature.get()));
//                    }
                    scene->UpdatePoints();
                }
                else
                {
                    leftPOV->GetCameraParameters().SetPoseDetermined(false);
                    rightPOV->GetCameraParameters().SetPoseDetermined(false);
                }

            }

            bool SceneReconstructor::EstimateCameraPose(const std::shared_ptr<Core::PointOfView> &pointOfView, cv::Mat &rotationMatrix, cv::Mat &translationMatrix)
            {
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

                if (pointCloud.size() < 5 || imagePoints.size() < 5)
                {
                    std::cout << "Not enough points to find pose: " << pointCloud.size() << std::endl;
                    return false;
                }

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
                    reprojectionErrors.push_back(norm(projectedPoints[i] - imagePoints[i]));
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
                return true;
            }


            void SceneReconstructor::AddForReconstruction(std::shared_ptr<Core::PointOfView> pointOfView)
            {
                if (!pointOfView->GetImage()->HasDepth())
                {
                    if (lastPointOfView == NULL)
                    {
                        pointOfView->GetCameraParameters().SetRotationMatrix(cv::Mat::eye(3, 3, CV_64FC1));
                        pointOfView->GetCameraParameters().SetTranslationMatrix(cv::Mat::zeros(3, 1, CV_64FC1));
                        scene->ResetFeatures();
                        features = scene->GetFeatures();
                        features->AddFeatures(pointOfView);
                        bundleAdjuster->Reset(features);

                        initialTriangulation = false;
                        lastPointOfView = pointOfView;
                        return;
                    }

                    if (lastPointOfView == pointOfView)
                    {
                        return;
                    }

                    currentPointOfView = pointOfView;

                    features->AddFeatures(currentPointOfView);

                    if (!initialTriangulation)
                    {
                        InitialReconstruction();

                        // DEBUGGING
//                        cv::Mat rot = currentPointOfView->GetCameraParameters().GetRotationMatrix();
//                        cv::Mat trans = currentPointOfView->GetCameraParameters().GetTranslationMatrix();

//                        cv::Mat rotationMatrix, translationMatrix;
//                        EstimateCameraPose(currentPointOfView, rotationMatrix, translationMatrix);

//                        currentPointOfView->GetCameraParameters().SetRotationMatrix(rotationMatrix);
//                        currentPointOfView->GetCameraParameters().SetTranslationMatrix(translationMatrix);

//                        bundleAdjuster->EstimateCameraPose(currentPointOfView);

//                        std::cout << "Expected:" << std::endl;
//                        std::cout << rot << std::endl;
//                        std::cout << trans << std::endl;

//                        std::cout << "New camera's' pose:" << std::endl;
//                        std::cout << currentPointOfView->GetCameraParameters().GetRotationMatrix() << std::endl;
//                        std::cout << currentPointOfView->GetCameraParameters().GetTranslationMatrix() << std::endl;

//                        currentPointOfView->GetCameraParameters().SetPoseDetermined(true);
                        // END DEBUGGING

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

//                    currentPointOfView->GetCameraParameters().SetRotationMatrix(rotationMatrix);
//                    currentPointOfView->GetCameraParameters().SetTranslationMatrix(translationMatrix);
//                    currentPointOfView->GetCameraParameters().SetPoseDetermined(true);

//                    currentPointOfView->GetCameraParameters().SetRotationMatrix(lastPointOfView->GetCameraParameters().GetRotationMatrix());
//                    currentPointOfView->GetCameraParameters().SetTranslationMatrix(lastPointOfView->GetCameraParameters().GetTranslationMatrix());
//                    bundleAdjuster->EstimateCameraPose(currentPointOfView);

//                    std::cout << "New camera's' pose:" << std::endl;
//                    std::cout << currentPointOfView->GetCameraParameters().GetRotationMatrix() << std::endl;
//                    std::cout << currentPointOfView->GetCameraParameters().GetTranslationMatrix() << std::endl;

                    currentPointOfView->GetCameraParameters().SetPoseDetermined(false);

//                    TriangulatePoints();

//                    double maxerr = 0.0;
//                    for (int i = 0; i < features->Size(); i++)
//                    {
//                        std::shared_ptr<Core::Feature> feature = features->GetFeature(i);
//                        if (feature->IsTriangulated() && !feature->IsHidden())
//                        {
//                            double error = feature->EstimateError(currentPointOfView);
//                            if (error > 10.0)
//                            {
//                                feature->SetHidden(true);
//                            }
//                            else if (error > maxerr)
//                            {
//                                maxerr = error;
//                            }
//                        }
//                        else if (!feature->IsTriangulated() && feature->GetCorrespondenceCount() > 5)
//                        {
//                            // We have 5 views and we haven't triangulated it yet... Remove.
//                            features->RemoveFeature(i);

//                        }
//                    }
//                    std::cout << maxerr << std::endl;

//                    shouldRunBundleAdjustment = true;
//                    bundleAdjuster->RunOnAllData();

                    scene->UpdatePoints();

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
