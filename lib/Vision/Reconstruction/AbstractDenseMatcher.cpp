#include "Vision/Reconstruction/AbstractDenseMatcher.h"

#include <limits>

#include <eigen3/Eigen/Geometry>

#include "Math/LinearAlgebra/Vector.h"
#include "Math/Calculus/Jacobian.h"

#include "Vision/Core/PointOfView.h"
#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {

            class ProjectionFunction : public Math::Calculus::MultivariateFunction<3, 2>
            {
                public:
                    ProjectionFunction(const std::shared_ptr<Core::PointOfView> &pointOfView)
                        : pointOfView(pointOfView)
                    {
                    }

                    Math::LinearAlgebra::Vector<2> Evaluate(Math::LinearAlgebra::Vector<3> point) const
                    {
                        return (pointOfView->GetCameraParameters().GetProjectionMatrix() * point.homogeneous()).hnormalized();
                    }

                private:
                    std::shared_ptr<Core::PointOfView> pointOfView;
            };


            AbstractDenseMatcher::AbstractDenseMatcher(std::size_t bundleSize, std::size_t iterations)
                : comparisonViews(bundleSize),
                  iterations(iterations)
            {
            }

            void AbstractDenseMatcher::ProcessImage(std::shared_ptr<Core::PointOfView> &pointOfView)
            {

                using namespace Math::LinearAlgebra;
                using namespace Math::Calculus;
//                let baseModel = assume known i.e. pcl triangulated featureset

                std::vector<cv::Mat> opticalFlow;
                std::vector<Jacobian<3, 2> > jacobians;

                for (const std::shared_ptr<Core::PointOfView> &otherPOV : comparisonViews)
                {
                    // PCL range image from features

                    opticalFlow.push_back(GetFlowMatrix(referenceView, otherPOV));

                    ProjectionFunction projectionFunction(otherPOV);
                    Jacobian<3, 2> jacobian(projectionFunction); // FIXME this will cause segfault
                    jacobians.push_back(jacobian);
                }

                #pragma omp parallel for
                for (int y = 0; y < referenceView->GetImage()->GetSize().height; y++)
                {
                    #pragma omp parallel for
                    for (int x = 0; x < referenceView->GetImage()->GetSize().width; x++)
                    {
                        Vector<2> pixel; pixel << x, y;
                        Vector<3> ray = (referenceView->GetCameraParameters().GetRotationMatrix() * pixel.homogeneous().transpose()).normalized();
                        Vector<3> point; // = intersection(ray, baseModel); // FIXME
                        double lambda = 0.0;

                        double error = std::numeric_limits<double>::infinity();

                        for (int i = 0; i < iterations && error > 1e-4; i++)
                        {
                            Vector<3> newPoint = point + lambda * ray;
                            Vector<RuntimeSized> Ks, dus;

                            for (int j = 0; j < comparisonViews.size(); j++)
                            {
                                const std::shared_ptr<Core::PointOfView> &otherPOV = comparisonViews.at(j);
                                Vector<3> projection = otherPOV->GetCameraParameters().GetProjectionMatrix() * newPoint;
                                projection.head<2>() /= projection(2);

                                Vector<2> u = projection.head<2>();
                                Vector<2> uf; uf << opticalFlow.at(j).at<cv::Vec2d>(y, x)[0], opticalFlow.at(j).at<cv::Vec2d>(y, x)[1];

                                Vector<2> du = u - uf;
                                Vector<2> K = jacobians.at(j).Evaluate(newPoint) * ray;

                                Ks(2 * j + 0) = K(0);
                                Ks(2 * j + 1) = K(1);
                                dus(2 * j + 0) = du(0);
                                dus(2 * j + 1) = du(1);
                            }

//                            solve for lambda: min(norm(Ks*lambda - dus));
//                            update lambda;
//                            update error;
                        }

                    }
                }

            }

        }
    }
}
