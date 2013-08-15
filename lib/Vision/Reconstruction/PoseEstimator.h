#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include <memory>

#include "Math/LinearAlgebra/Matrix.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Point;
            class PointOfView;
            class Projection;
            class Scene;
        }

        namespace Reconstruction
        {
            class PoseEstimator
            {
                public:
                    PoseEstimator(const std::shared_ptr<Core::Scene> &scene);

                    void EstimateCameraPose(std::shared_ptr<Core::PointOfView> &pointOfView);

                private:
                    typedef std::pair<Core::Point, Core::Projection> DataType;
                    typedef Math::LinearAlgebra::Matrix<3, 4> ParametersType;

                    ParametersType EstimateProjection(const std::vector<DataType> &data);
                    Math::Core::Number EstimateError(const ParametersType &projectionMatrix, const DataType &datum);

                    std::shared_ptr<Core::Scene> scene;
            };
        }
    }
}

#endif // POSEESTIMATOR_H
