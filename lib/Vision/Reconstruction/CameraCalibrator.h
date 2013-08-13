#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class PointOfView;
        }

        namespace Reconstruction
        {
            class CameraCalibrator
            {
                public:
                    CameraCalibrator();

                    void AddPointOfView(std::shared_ptr<Core::PointOfView> pointOfView);
                    cv::Mat GetIntrinsicParameters() const;

                private:
                    void SolveKruppaEquations();

                    cv::Mat kruppaCoefficients;

                    std::vector<std::shared_ptr<Core::PointOfView> > pointsOfView;
                    std::map<std::pair<int, int>, cv::SVD> fundamentalMatrixPairs;
                    std::vector<cv::Mat> svds;
            };

        }
    }
}

#endif // CAMERACALIBRATOR_H
