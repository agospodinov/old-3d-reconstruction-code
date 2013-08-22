#include "Vision/Core/CameraParameters.h"

#include <Eigen/LU>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            CameraParameters::CameraParameters()
                : cameraMatrix(Math::LinearAlgebra::Matrix<3, 3>::Identity()),
                  distortionCoefficients(Math::LinearAlgebra::Vector<4>::Zero()),
                  rotationMatrix(Math::LinearAlgebra::Matrix<3, 3>::Identity()),
                  translationMatrix(Math::LinearAlgebra::Vector<3>::Zero()),
                  cameraMatrixKnown(false),
                  poseDetermined(false)
            {
            }

            const Math::LinearAlgebra::Matrix<3, 3> &CameraParameters::GetCameraMatrix() const
            {
                return cameraMatrix;
            }

            const Math::LinearAlgebra::Matrix<3, 3> CameraParameters::GetInverseCameraMatrix() const
            {
                return cameraMatrix.inverse();
            }

            void CameraParameters::SetCameraMatrix(const Math::LinearAlgebra::Matrix<3, 3> &cameraMatrix)
            {
                this->cameraMatrix = cameraMatrix;
            }

            double CameraParameters::GetFocalLength() const
            {
                // This isn't necessarily true for all cameras,
                // but it simplifies the problem.
                assert (cameraMatrix(0, 0) == cameraMatrix(1, 1));
                return cameraMatrix(0, 0);
            }

            void CameraParameters::SetFocalLength(double focalLength)
            {
                cameraMatrix(0, 0) = cameraMatrix(1, 1) = focalLength;
            }

            const Math::LinearAlgebra::Vector<Math::LinearAlgebra::RuntimeSized> &CameraParameters::GetDistortionCoefficients() const
            {
                return distortionCoefficients;
            }

            void CameraParameters::SetDistortionCoefficients(const Math::LinearAlgebra::Vector<Math::LinearAlgebra::RuntimeSized> &distortionCoefficients)
            {
                assert(distortionCoefficients.rows() == 2
                       || distortionCoefficients.rows() == 4
                       || distortionCoefficients.rows() == 5
                       || distortionCoefficients.rows() == 8);
                this->distortionCoefficients = distortionCoefficients;
            }

            Math::LinearAlgebra::Matrix<3, 4> CameraParameters::GetPoseMatrix() const
            {
                Math::LinearAlgebra::Matrix<3, 4> poseMatrix;
                poseMatrix << rotationMatrix, translationMatrix;
                return poseMatrix;
            }

            void CameraParameters::SetPoseMatrix(const Math::LinearAlgebra::Matrix<3, 4> &poseMatrix)
            {
                rotationMatrix = poseMatrix.block(0, 0, 3, 3);
                translationMatrix = poseMatrix.col(3);
            }

            const Math::LinearAlgebra::Matrix<3, 3> &CameraParameters::GetRotationMatrix() const
            {
                return rotationMatrix;
            }

            void CameraParameters::SetRotationMatrix(const Math::LinearAlgebra::Matrix<3, 3> &rotationMatrix)
            {
                this->rotationMatrix = rotationMatrix;
            }

            const Math::LinearAlgebra::Vector<3> &CameraParameters::GetTranslationMatrix() const
            {
                return translationMatrix;
            }

            void CameraParameters::SetTranslationMatrix(const Math::LinearAlgebra::Vector<3> &translationMatrix)
            {
                this->translationMatrix = translationMatrix;
            }

            Math::LinearAlgebra::Matrix<3, 4> CameraParameters::GetProjectionMatrix() const
            {
                return GetCameraMatrix() * GetPoseMatrix();
            }

            bool CameraParameters::IsCameraMatrixKnown() const
            {
                return cameraMatrixKnown;
            }

            void CameraParameters::SetCameraMatrixKnown(bool cameraMatrixKnown)
            {
                this->cameraMatrixKnown = cameraMatrixKnown;
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
