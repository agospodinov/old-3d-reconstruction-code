#include "Vision/Core/Point.h"

#include <tuple>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>

#include <opencv2/core/eigen.hpp>

#include "Vision/Core/PointOfView.h"
#include "Vision/Core/Projection.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            namespace
            {
                template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
                struct LMDifFunctor
                {
                        typedef _Scalar Scalar;
                        enum {
                            InputsAtCompileTime = NX,
                            ValuesAtCompileTime = NY
                        };
                        typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
                        typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
                        typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

                        const int m_inputs, m_values;
                        const std::vector<Projection> *usefulProjections;

                        LMDifFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
                        LMDifFunctor(int inputs, int values, const std::vector<Projection> *usefulProjections)
                            : m_inputs(inputs),
                              m_values(values),
                              usefulProjections(usefulProjections)
                        {
                        }

                        int inputs() const { return m_inputs; }
                        int values() const { return m_values; }

                        int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
                        {
                            int i = 0;
                            for (const Projection &projection : *usefulProjections)
                            {
                                std::shared_ptr<PointOfView> pointOfView = projection.GetPointOfView();

                                // Refactor, use Point's implementation.
                                double dx, dy;

                                Math::LinearAlgebra::Vector<4> point;
                                point << x(0), x(1), x(2), 1;
                                Math::LinearAlgebra::Matrix<3, 4> P = pointOfView->GetCameraParameters().GetProjectionMatrix();

                                Math::LinearAlgebra::Vector<3> projectedPoint = P * point;
                                projectedPoint.head<2>() /= projectedPoint(2);

                                dx = projectedPoint(0) - projection.GetX();
                                dy = projectedPoint(1) - projection.GetY();

                                fvec[2 * i + 0] = dx;
                                fvec[2 * i + 1] = dy;

                                i++;
                            }

                            return 0;
                        }
                };
            }

            Point::Point()
                : x(0), y(0), z(0),
                  r(0), g(0), b(0),
                  triangulated(false)
            {
            }

            Point::~Point()
            {
            }

//            pcl::PointXYZRGB Point::GetPCLPoint() const
//            {
//                pcl::PointXYZRGB point;
//                point.x = (float) x; point.y = (float) y; point.z = (float) z;
//                uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//                point.rgb = *reinterpret_cast<float*>(&rgb);
//                return point;
//            }

            double Point::GetX() const
            {
                return x;
            }

            double Point::GetY() const
            {
                return y;
            }

            double Point::GetZ() const
            {
                return z;
            }

            void Point::SetPosition(double x, double y, double z)
            {
                this->x = x;
                this->y = y;
                this->z = z;
            }

            uchar Point::GetR() const
            {
                return r;
            }

            uchar Point::GetG() const
            {
                return g;
            }

            uchar Point::GetB() const
            {
                return b;
            }

            void Point::SetColor(uchar r, uchar g, uchar b)
            {
                this->r = r;
                this->g = g;
                this->b = b;
            }

            bool Point::IsTriangulated() const
            {
                return triangulated;
            }

            const std::vector<Projection> &Point::GetProjections() const
            {
                return projections;
            }

            const boost::optional<Projection> Point::GetProjection(const std::shared_ptr<PointOfView> &pointOfView) const
            {
                auto it = std::find_if(projections.begin(), projections.end(), [&](const Projection &projection)
                {
                    // By definition, shared pointers should not point to
                    // null before all of them fall out of scope. Having been
                    // passed a reference to a shared ptr, we can assume that
                    // the object is not null, therefore even if the projection's
                    // point of view has expired, null != pointOfView.
                    return projection.GetPointOfView() == pointOfView;
                });

                if (it != projections.end())
                {
                    return boost::optional<Projection>(*it);
                }
                return boost::optional<Projection>();
            }

            bool Point::HasProjection(const std::shared_ptr<PointOfView> &pointOfView) const
            {
                return GetProjection(pointOfView).is_initialized();
            }

            void Point::AddProjection(const Projection &projection)
            {
                projections.push_back(projection);
            }

            void Point::RemoveProjection(const std::shared_ptr<PointOfView> &pointOfView)
            {
                projections.erase(std::remove_if(projections.begin(), projections.end(), [&](const Projection &projection)
                {
                    return projection.GetPointOfView() == pointOfView;
                }), projections.end());
            }

            void Point::Triangulate(bool reset, bool optimize)
            {
                std::vector<Projection> usefulProjections;
                std::copy_if(projections.begin(), projections.end(), std::back_inserter(usefulProjections), [](const Projection &projection)
                {
                    std::shared_ptr<PointOfView> pointOfView = projection.GetPointOfView();
                    return (pointOfView != NULL && pointOfView->GetCameraParameters().IsPoseDetermined());
                });

                if (!triangulated || (triangulated && reset))
                {
                    triangulated = false;

                    if (usefulProjections.size() < 2)
                    {
                        return;
                    }

                    TriangulateLinear(usefulProjections);

                    triangulated = true;
                }

                if (triangulated && optimize)
                {
                    Refine3DPosition(usefulProjections);
                }

                double totalError = 0.0;
                for (const Projection &actualProjection : usefulProjections)
                {
                    double error = EstimateError(actualProjection.GetPointOfView()).get();

                    // Be brutal about any outlying points
//                    if (error > 8.0)
//                    {
//                        RemoveProjection(actualProjection.GetPointOfView());
//                    }

                    totalError += error;
                }
                totalError /= static_cast<double>(usefulProjections.size());

                // Be brutal about badly triangulated points
                if (totalError > 16.0)
                {
                    triangulated = false;
                }

            }

            Projection Point::ProjectPoint(const std::shared_ptr<PointOfView> &pointOfView)
            {
                Math::LinearAlgebra::Matrix<3, 4> P = pointOfView->GetCameraParameters().GetProjectionMatrix();

                // TODO: change to:
                // Vector3d point; Vector2d projectedPoint = (P * point.homogeneous()).hnormalized();
                Eigen::Vector4d point; point << x, y, z, 1;
                Eigen::Vector3d projectedPoint;
                projectedPoint = P * point;
                projectedPoint.head<2>() /= projectedPoint(2);

                return Projection(projectedPoint(0), projectedPoint(1), pointOfView, this, false);
            }

            boost::optional<double> Point::EstimateError(const std::shared_ptr<PointOfView> &pointOfView)
            {
                boost::optional<double> distance;
                boost::optional<Projection> projection = GetProjection(pointOfView);
                if (projection.is_initialized())
                {
                    double dx, dy;

                    Projection projectedPoint = ProjectPoint(pointOfView);

                    dx = projectedPoint.GetX() - projection->GetX();
                    dy = projectedPoint.GetY() - projection->GetY();

                    distance = std::sqrt(dx * dx + dy * dy);
                }
                return distance;
            }

            bool operator ==(const Point &left, const Point &right)
            {
                return (left.x == right.x && left.y == right.y && left.z == right.z
                        && left.r == right.r && left.g == right.g && left.b == right.b
                        && left.triangulated == right.triangulated
                        && left.projections == right.projections);
            }

            bool operator <(const Point &left, const Point &right)
            {
                return std::tie(left.x, left.y, left.z,
                                left.r, left.g, left.b,
                                left.triangulated,
                                left.projections) <
                        std::tie(right.x, right.y, right.z,
                                 right.r, right.g, right.b,
                                 right.triangulated,
                                 right.projections);
            }

            void Point::TriangulateLinear(const std::vector<Projection> &usefulProjections)
            {
                int usefulPointsOfViewCount = usefulProjections.size();
                Eigen::MatrixXd A(2 * usefulPointsOfViewCount, 3);
                Eigen::MatrixXd B(2 * usefulPointsOfViewCount, 1);

                int i = 0;
                for (const Projection &projection : usefulProjections)
                {
                    std::shared_ptr<PointOfView> pointOfView = projection.GetPointOfView();

                    Eigen::Vector3d p(projection.GetX(), projection.GetY(), -1.0);
                    Eigen::Vector3d pi = pointOfView->GetCameraParameters().GetInverseCameraMatrix() * p;
                    pi.head<2>() /= pi(2);

                    Eigen::MatrixXd R = pointOfView->GetCameraParameters().GetRotationMatrix();
                    Eigen::MatrixXd T = pointOfView->GetCameraParameters().GetTranslationMatrix();

                    A(2 * i + 0, 0) = R(0, 0) - pi(0) * R(2, 0);
                    A(2 * i + 0, 1) = R(0, 1) - pi(0) * R(2, 1);
                    A(2 * i + 0, 2) = R(0, 2) - pi(0) * R(2, 2);

                    A(2 * i + 1, 0) = R(1, 0) - pi(1) * R(2, 0);
                    A(2 * i + 1, 1) = R(1, 1) - pi(1) * R(2, 1);
                    A(2 * i + 1, 2) = R(1, 2) - pi(1) * R(2, 2);

                    B(2 * i + 0, 0) = pi(0) * T(2) - T(0);
                    B(2 * i + 1, 0) = pi(1) * T(2) - T(1);

                    i++;
                }

                Eigen::MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

                this->x = X(0);
                this->y = X(1);
                this->z = X(2);
            }

            void Point::Refine3DPosition(const std::vector<Projection> &usefulProjections)
            {
                const int m = 2 * usefulProjections.size(), n = 3;
                int info;
                Eigen::VectorXd point(3);
                point(0) = x;
                point(1) = y;
                point(2) = z;

                LMDifFunctor<double> functor(n, m, &usefulProjections);
                Eigen::NumericalDiff<LMDifFunctor<double> > numDiff(functor);
                Eigen::LevenbergMarquardt<Eigen::NumericalDiff<LMDifFunctor<double> > > lm(numDiff);
                info = lm.minimize(point);

                x = point(0);
                y = point(1);
                z = point(2);
            }
        }
    }
}
