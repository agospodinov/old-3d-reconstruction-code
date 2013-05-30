#include "Vision/Reconstruction/CameraCalibrator.h"

namespace Xu
{
    namespace Vision
    {

        namespace Reconstruction
        {

            CameraCalibrator::CameraCalibrator()
            {
            }

            void CameraCalibrator::AddPointOfView(std::shared_ptr<Core::PointOfView> pointOfView)
            {
                // stub
            }

            cv::Mat CameraCalibrator::GetIntrinsicParameters() const
            {
                // stub
            }

            void CameraCalibrator::SolveKruppaEquations()
            {
                // stub
            }

        }
    }
}
