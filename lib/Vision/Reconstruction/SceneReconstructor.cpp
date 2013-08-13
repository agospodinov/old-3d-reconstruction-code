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
#include "Vision/Reconstruction/PoseEstimator.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            SceneReconstructor::SceneReconstructor(std::shared_ptr<Core::SingleViewCamera> camera)
                : camera(camera),
                  scene(new Core::Scene()),
//                  featureMatcher(new SURFGPUFeatureMatcher(*scene)),
                  featureMatcher(new GFTTFeatureMatcher(*scene)),
                  bundleAdjuster(new BundleAdjuster(*scene)),
                  denseMatcher(new DenseMatcher(scene)),
                  poseEstimator(new PoseEstimator(scene)),
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

                    poseEstimator->EstimateCameraPose(currentPointOfView);

                    if (!currentPointOfView->GetCameraParameters().IsPoseDetermined())
                    {
                        std::cout << "Failed to estimate camera pose." << std::endl;

                        lastPointOfView = currentPointOfView;

                        return;
                    }

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
