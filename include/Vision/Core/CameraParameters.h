#ifndef CAMERAPARAMETERS_H
#define CAMERAPARAMETERS_H

#include <opencv2/core/core.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            // TODO replace all cv::Mat with Math::Matrix
            class CameraParameters
            {
                public:
                    CameraParameters();

                    const cv::Mat &GetCameraMatrix() const;
                    const cv::Mat &GetInverseCameraMatrix() const;
                    void SetCameraMatrix(const cv::Mat &cameraMatrix);

                    double GetFocalLength() const;
                    void SetFocalLength(double focalLength);

                    const cv::Mat &GetDistortionCoefficients() const;
                    void SetDistortionCoefficients(const cv::Mat &distortionCoefficients);

                    cv::Mat GetPoseMatrix() const;
                    void SetPoseMatrix(const cv::Mat &poseMatrix);

                    const cv::Mat &GetRotationMatrix() const;
                    void SetRotationMatrix(const cv::Mat &rotationMatrix);

                    const cv::Mat &GetTranslationMatrix() const;
                    void SetTranslationMatrix(const cv::Mat &translationMatrix);

                    cv::Mat GetProjectionMatrix() const;

                    bool IsCameraMatrixKnown() const;
                    void SetCameraMatrixKnown(bool cameraMatrixKnown);

                    bool IsPoseDetermined() const;
                    void SetPoseDetermined(bool poseDetermined);

                private:
                    cv::Mat cameraMatrix;
                    cv::Mat inverseCameraMatrix;

                    cv::Mat distortionCoefficients;

                    cv::Mat translationMatrix;
                    cv::Mat rotationMatrix;

                    bool cameraMatrixKnown;
                    bool poseDetermined;
            };

        }
    }
}

#endif // CAMERAPARAMETERS_H
