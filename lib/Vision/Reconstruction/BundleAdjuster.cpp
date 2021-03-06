#include "Vision/Reconstruction/BundleAdjuster.h"

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <sba.h>

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
            BundleAdjuster::BundleAdjuster()
                : adjustedCameras(0),
                  adjustedPoints(0)
            {
            }

            BundleAdjuster::BundleAdjuster(const std::shared_ptr<Core::Scene> &scene)
                : scene(scene),
                  adjustedCameras(0),
                  adjustedPoints(0)
            {
            }

            void BundleAdjuster::EstimateCameraPose(std::shared_ptr<Core::PointOfView> &pointOfView)
            {
                SbaParameters params = PrepareData(MOTION_ONLY);
                params.pointsOfView.push_back(pointOfView);

                Run(0, params.pointsOfView.size() - 1, params);
            }

            void BundleAdjuster::RunOnNewData()
            {
                SbaParameters params = PrepareData(STRUCTURE_AND_MOTION);
                Run(adjustedPoints, adjustedCameras, params);
            }

            void BundleAdjuster::RunOnAllData()
            {
                SbaParameters params = PrepareData(STRUCTURE_AND_MOTION);
                Run(0, 0, params);
            }

            void BundleAdjuster::Reset(const std::shared_ptr<Core::Scene> &scene)
            {
                this->scene = scene;
            }

            BundleAdjuster::SbaParameters BundleAdjuster::PrepareData(SbaMode mode)
            {
                SbaParameters params(this, mode);

                for (auto it = scene->GetFeatures()->Begin(); it != scene->GetFeatures()->End(); it++)
                {
                    Core::Feature &feature = *it;
                    if (feature.IsTriangulated())
                    {
                        params.triangulatedPoints.push_back(&feature);
                    }
                }

                // FIXME
                for (const std::shared_ptr<Core::PointOfView> &pointOfView : pointsOfView)
                {
                    if (pointOfView->GetCameraParameters().IsPoseDetermined())
                    {
                        params.pointsOfView.push_back(pointOfView);
                    }
                }

                return params;
            }

            void BundleAdjuster::Run(int startingPoint, int startingCamera, SbaParameters &params)
            {
                int pointCount = params.triangulatedPoints.size();

                if (startingPoint > pointCount)
                {
                    startingPoint = 0;
                }

                int cameraCount = params.pointsOfView.size();

                if (startingCamera > cameraCount)
                {
                    startingCamera = 0;
                }

                char *visibility = new char[pointCount * cameraCount];
                double *projections = new double[2 * pointCount * cameraCount];

                for (int i = 0; i < pointCount * cameraCount; i++)
                {
                    visibility[i] = 0;

                    projections[2 * i + 0] = 0;
                    projections[2 * i + 1] = 0;
                }

                for (int i = 0; i < cameraCount; i++)
                {
                    const std::shared_ptr<Core::PointOfView> &pointOfView = params.pointsOfView.at(i);
                    for (int j = 0; j < pointCount; j++)
                    {
                        Core::Feature *feature = params.triangulatedPoints.at(j);

                        boost::optional<Core::Projection> projection = feature->GetProjection(pointOfView);
                        if (projection.is_initialized())
                        {
                            visibility[j * cameraCount + i] = 1;

                            projections[2 * j * cameraCount + 2 * i + 0] = projection->GetX();
                            projections[2 * j * cameraCount + 2 * i + 1] = projection->GetY();
                        }
                    }
                }

                const int cameraParametersCount = 7;
                const int pointParametersCount = 3;
                const int parametersCount = cameraCount * cameraParametersCount + pointCount * pointParametersCount;

                double *parameters = new double[parametersCount];

                for (int i = 0; i < cameraCount; i++)
                {
                    const std::shared_ptr<Core::PointOfView> &pointOfView = params.pointsOfView.at(i);
                    Math::LinearAlgebra::Matrix<3, 3> rotationMatrix = pointOfView->GetCameraParameters().GetRotationMatrix();
                    cv::Mat rmat(3, 3, CV_64FC1), rvec;
                    cv::eigen2cv(rotationMatrix, rmat);
                    cv::Rodrigues(rmat, rvec);

                    Math::LinearAlgebra::Vector<3> translation = pointOfView->GetCameraParameters().GetTranslationMatrix();

                    parameters[i * cameraParametersCount + 0] = translation(0);
                    parameters[i * cameraParametersCount + 1] = translation(1);
                    parameters[i * cameraParametersCount + 2] = translation(2);

                    parameters[i * cameraParametersCount + 3] = rvec.at<double>(0);
                    parameters[i * cameraParametersCount + 4] = rvec.at<double>(1);
                    parameters[i * cameraParametersCount + 5] = rvec.at<double>(2);

                    parameters[i * cameraParametersCount + 6] = pointOfView->GetCameraParameters().GetFocalLength();
                }

                int pointsStartIndex = cameraCount * cameraParametersCount;

                for (int i = 0; i < pointCount; i++)
                {
                    const Core::Feature *feature = params.triangulatedPoints.at(i);

                    parameters[pointsStartIndex + i * pointParametersCount + 0] = feature->GetX();
                    parameters[pointsStartIndex + i * pointParametersCount + 1] = feature->GetY();
                    parameters[pointsStartIndex + i * pointParametersCount + 2] = feature->GetZ();
                }

                double options[SBA_OPTSSZ];
                options[0] = SBA_INIT_MU;
                options[1] = SBA_STOP_THRESH;
                options[2] = SBA_STOP_THRESH;
                options[3] = SBA_STOP_THRESH;
                options[4] = 0.0;
                double info[SBA_INFOSZ];

                switch (params.mode) {
                    case STRUCTURE_AND_MOTION:
                        sba_motstr_levmar(pointCount,
                                          startingPoint,
                                          cameraCount,
                                          startingCamera,
                                          visibility,
                                          parameters,
                                          cameraParametersCount,
                                          pointParametersCount,
                                          projections,
                                          NULL, // covariance matrix
                                          2,
                                          ProjectPoint,
                                          NULL, // jacobian calculation function
                                          &params, // additional data
                                          150, // max iterations
                                          3, // verbosity level
                                          options,
                                          info);
                        break;
                    case STRUCTURE_ONLY:
                        sba_str_levmar(pointCount,
                                       startingPoint,
                                       cameraCount,
                                       visibility,
                                       parameters,
                                       pointParametersCount,
                                       projections,
                                       NULL, // covariance matrix
                                       2,
                                       ProjectPointStructureOnly,
                                       NULL, // jacobian calculation function
                                       &params, // additional data
                                       150, // max iterations
                                       3, // verbosity level
                                       options,
                                       info);
                        break;
                    case MOTION_ONLY:
                        sba_mot_levmar(pointCount,
                                       cameraCount,
                                       startingCamera,
                                       visibility,
                                       parameters,
                                       cameraParametersCount,
                                       projections,
                                       NULL, // covariance matrix
                                       2,
                                       ProjectPointMotionOnly,
                                       NULL, // jacobian calculation function
                                       &params, // additional data
                                       150, // max iterations
                                       3, // verbosity level
                                       options,
                                       info);
                        break;
                }

                delete[] visibility;
                delete[] projections;

                for (int i = startingCamera; i < cameraCount; i++)
                {
                    std::shared_ptr<Core::PointOfView> &pointOfView = params.pointsOfView.at(i);

                    Math::LinearAlgebra::Vector<3> translationMatrix;
                    translationMatrix(0) = parameters[i * cameraParametersCount + 0];
                    translationMatrix(1) = parameters[i * cameraParametersCount + 1];
                    translationMatrix(2) = parameters[i * cameraParametersCount + 2];

                    cv::Mat rvec(3, 1, CV_64FC1), rmat;
                    rvec.at<double>(0) = parameters[i * cameraParametersCount + 3];
                    rvec.at<double>(1) = parameters[i * cameraParametersCount + 4];
                    rvec.at<double>(2) = parameters[i * cameraParametersCount + 5];
                    Math::LinearAlgebra::Matrix<3, 3> rotationMatrix;
                    cv::Rodrigues(rvec, rmat);
                    cv::cv2eigen(rmat, rotationMatrix);

                    double focalLength = parameters[i * cameraParametersCount + 6];
                    std::cout << "Focal length estimate for camera " << i << ": " << focalLength << std::endl;

                    pointOfView->GetCameraParameters().SetRotationMatrix(rotationMatrix);
                    pointOfView->GetCameraParameters().SetTranslationMatrix(translationMatrix);

                    pointOfView->GetCameraParameters().SetFocalLength(focalLength);
                }

                for (int i = startingPoint; i < pointCount; i++)
                {
                    Core::Feature *feature = params.triangulatedPoints.at(i);

                    double x = parameters[pointsStartIndex + i * pointParametersCount + 0];
                    double y = parameters[pointsStartIndex + i * pointParametersCount + 1];
                    double z = parameters[pointsStartIndex + i * pointParametersCount + 2];

                    feature->SetPosition(x, y, z);
                }

                delete[] parameters;

                adjustedCameras = cameraCount;
                adjustedPoints = pointCount;
            }

            void BundleAdjuster::ProjectPoint(int j, int i, double *cameraParams, double *pointParams, double *projection, void *additionalData)
            {
                using namespace Math::LinearAlgebra;
                SbaParameters *params = static_cast<SbaParameters *>(additionalData);

                Vector<3> translation;
                translation << cameraParams[0], cameraParams[1], cameraParams[2];

                cv::Mat rvec(3, 1, CV_64FC1), rmat;
                rvec.at<double>(0) = cameraParams[3];
                rvec.at<double>(1) = cameraParams[4];
                rvec.at<double>(2) = cameraParams[5];
                Matrix<3, 3> rotationMatrix;
                cv::Rodrigues(rvec, rmat);
                cv::cv2eigen(rmat, rotationMatrix);

                Matrix<3, 4> P;
                P << rotationMatrix, translation;

                double focalLength = cameraParams[6];
                Matrix<3, 3> K = params->pointsOfView.at(j)->GetCameraParameters().GetCameraMatrix();
                K(0, 0) = K(1, 1) = focalLength;

                Vector<4> point;
                point << pointParams[0], pointParams[1], pointParams[2], 1;

                Vector<3> projectedPoint = K * P * point;
                projectedPoint.head<2>() /= projectedPoint(2);

                projection[0] = projectedPoint(0);
                projection[1] = projectedPoint(1);
            }

            void BundleAdjuster::ProjectPointStructureOnly(int j, int i, double *pointParams, double *projection, void *additionalData)
            {
                using namespace Math::LinearAlgebra;
                SbaParameters *params = static_cast<SbaParameters *>(additionalData);

                double cameraParams[7];
                cameraParams[0] = params->pointsOfView.at(j)->GetCameraParameters().GetTranslationMatrix()(0);
                cameraParams[1] = params->pointsOfView.at(j)->GetCameraParameters().GetTranslationMatrix()(1);
                cameraParams[2] = params->pointsOfView.at(j)->GetCameraParameters().GetTranslationMatrix()(2);

                Matrix<3, 3> rotationMatrix = params->pointsOfView.at(j)->GetCameraParameters().GetRotationMatrix();
                cv::Mat rmat(3, 3, CV_64FC1), rvec;
                cv::eigen2cv(rotationMatrix, rmat);
                cv::Rodrigues(rmat, rvec);

                cameraParams[3] = rvec.at<double>(0);
                cameraParams[4] = rvec.at<double>(1);
                cameraParams[5] = rvec.at<double>(2);

                cameraParams[6] = params->pointsOfView.at(j)->GetCameraParameters().GetFocalLength();

                ProjectPoint(j, i, cameraParams, pointParams, projection, additionalData);
            }

            void BundleAdjuster::ProjectPointMotionOnly(int j, int i, double *cameraParams, double *projection, void *additionalData)
            {
                SbaParameters *params = static_cast<SbaParameters *>(additionalData);

                double pointParams[3];
                pointParams[0] = params->triangulatedPoints.at(i)->GetX();
                pointParams[1] = params->triangulatedPoints.at(i)->GetY();
                pointParams[2] = params->triangulatedPoints.at(i)->GetZ();

                ProjectPoint(j, i, cameraParams, pointParams, projection, additionalData);
            }
        }
    }
}
