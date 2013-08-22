#ifndef CAMERAPARAMETERS_H
#define CAMERAPARAMETERS_H

#include "Math/LinearAlgebra/Matrix.h"
#include "Math/LinearAlgebra/Vector.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class CameraParameters
            {
                public:
                    CameraParameters();

                    const Math::LinearAlgebra::Matrix<3, 3> &GetCameraMatrix() const;
                    const Math::LinearAlgebra::Matrix<3, 3> GetInverseCameraMatrix() const;
                    void SetCameraMatrix(const Math::LinearAlgebra::Matrix<3, 3> &cameraMatrix);

                    double GetFocalLength() const;
                    void SetFocalLength(double focalLength);

                    const Math::LinearAlgebra::Vector<Math::LinearAlgebra::RuntimeSized> &GetDistortionCoefficients() const;
                    void SetDistortionCoefficients(const Math::LinearAlgebra::Vector<Math::LinearAlgebra::RuntimeSized> &distortionCoefficients);

                    Math::LinearAlgebra::Matrix<3, 4> GetPoseMatrix() const;
                    void SetPoseMatrix(const Math::LinearAlgebra::Matrix<3, 4> &poseMatrix);

                    const Math::LinearAlgebra::Matrix<3, 3> &GetRotationMatrix() const;
                    void SetRotationMatrix(const Math::LinearAlgebra::Matrix<3, 3> &rotationMatrix);

                    const Math::LinearAlgebra::Vector<3> &GetTranslationMatrix() const;
                    void SetTranslationMatrix(const Math::LinearAlgebra::Vector<3> &translationMatrix);

                    Math::LinearAlgebra::Matrix<3, 4> GetProjectionMatrix() const;

                    bool IsCameraMatrixKnown() const;
                    void SetCameraMatrixKnown(bool cameraMatrixKnown);

                    bool IsPoseDetermined() const;
                    void SetPoseDetermined(bool poseDetermined);

                private:
                    Math::LinearAlgebra::Matrix<3, 3> cameraMatrix;
                    Math::LinearAlgebra::Vector<Math::LinearAlgebra::RuntimeSized> distortionCoefficients;

                    Math::LinearAlgebra::Matrix<3, 3> rotationMatrix;
                    Math::LinearAlgebra::Vector<3> translationMatrix;

                    bool cameraMatrixKnown;
                    bool poseDetermined;
            };

        }
    }
}

#endif // CAMERAPARAMETERS_H
