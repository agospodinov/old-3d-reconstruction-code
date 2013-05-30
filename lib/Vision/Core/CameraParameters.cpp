#include "Vision/Core/CameraParameters.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            CameraParameters::CameraParameters()
                : cameraMatrix(cv::Mat::eye(3, 3, CV_64FC1)),
                  inverseCameraMatrix(cv::Mat::eye(3, 3, CV_64FC1)),
                  distortionCoefficients(cv::Mat::zeros(4, 1, CV_64FC1)),
                  rotationMatrix(cv::Mat::eye(3, 3, CV_64FC1)),
                  translationMatrix(cv::Mat::zeros(3, 1, CV_64FC1)),
                  cameraMatrixKnown(false),
                  poseDetermined(false)
            {
            }

            const cv::Mat &CameraParameters::GetCameraMatrix() const
            {
                return cameraMatrix;
            }

            const cv::Mat &CameraParameters::GetInverseCameraMatrix() const
            {
                return inverseCameraMatrix;
            }

            void CameraParameters::SetCameraMatrix(const cv::Mat &cameraMatrix)
            {
                this->cameraMatrix = cameraMatrix;
                cv::invert(this->cameraMatrix, this->inverseCameraMatrix);
            }

            double CameraParameters::GetFocalLength() const
            {
                // This isn't necessarily true for all cameras,
                // but it simplifies the problem.
                assert (cameraMatrix.at<double>(0, 0) == cameraMatrix.at<double>(1, 1));
                return cameraMatrix.at<double>(0, 0);
            }

            void CameraParameters::SetFocalLength(double focalLength)
            {
                cameraMatrix.at<double>(0, 0) = focalLength;
                cameraMatrix.at<double>(1, 1) = focalLength;
                cv::invert(this->cameraMatrix, this->inverseCameraMatrix);
            }

            const cv::Mat &CameraParameters::GetDistortionCoefficients() const
            {
                return distortionCoefficients;
            }

            void CameraParameters::SetDistortionCoefficients(const cv::Mat &distortionCoefficients)
            {
                this->distortionCoefficients = distortionCoefficients;
            }

            cv::Mat CameraParameters::GetPoseMatrix() const
            {
                cv::Mat poseMatrix;
                hconcat(rotationMatrix, translationMatrix, poseMatrix);
                CV_Assert(poseMatrix.rows == 3 && poseMatrix.cols == 4);
                return poseMatrix;
            }

            void CameraParameters::SetPoseMatrix(const cv::Mat &poseMatrix)
            {
                CV_Assert(poseMatrix.rows == 3 && poseMatrix.cols == 4);
                rotationMatrix = poseMatrix.colRange(0, 2);
                translationMatrix = poseMatrix.col(3);
            }

            const cv::Mat &CameraParameters::GetRotationMatrix() const
            {
                return rotationMatrix;
            }

            void CameraParameters::SetRotationMatrix(const cv::Mat &rotationMatrix)
            {
                this->rotationMatrix = rotationMatrix;
            }

            const cv::Mat &CameraParameters::GetTranslationMatrix() const
            {
                return translationMatrix;
            }

            void CameraParameters::SetTranslationMatrix(const cv::Mat &translationMatrix)
            {
                this->translationMatrix = translationMatrix;
            }

            cv::Mat CameraParameters::GetProjectionMatrix() const
            {
                return GetCameraMatrix() * GetPoseMatrix();
            }

            bool CameraParameters::IsCameraMatrixKnown() const
            {
                return cameraMatrixKnown;
            }

            void CameraParameters::SetCameraMatrixKnown(bool cameraMatrixKnown)
            {
                this->cameraMatrix = cameraMatrixKnown;
            }

            bool CameraParameters::IsPoseDetermined() const
            {
                return poseDetermined;
            }

            void CameraParameters::SetPoseDetermined(bool poseDetermined)
            {
                this->poseDetermined = poseDetermined;
            }

        }
    }
}
