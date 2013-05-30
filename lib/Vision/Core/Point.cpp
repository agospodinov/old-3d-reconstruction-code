#include "Vision/Core/Point.h"

#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>

#include "Vision/Core/PointOfView.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
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
                    const Point *point;

                    LMDifFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
                    LMDifFunctor(int inputs, int values, const Point *point)
                        : m_inputs(inputs),
                          m_values(values),
                          point(point)
                    {
                    }

                    int inputs() const { return m_inputs; }
                    int values() const { return m_values; }

                    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
                    {
                        std::vector<std::shared_ptr<PointOfView> > usefulPointsOfView = point->GetUsefulPointsOfView();
                        int usefulPointsOfViewCount = usefulPointsOfView.size();

                        if (usefulPointsOfViewCount < 2)
                        {
                            return 1;
                        }

                        for (int i = 0; i < usefulPointsOfViewCount; i++)
                        {
                            std::shared_ptr<PointOfView> pointOfView = usefulPointsOfView[i];
                            cv::Point2d imagePoint = point->GetPointInView(pointOfView);
                            double dx, dy;

                            cv::Point3d p(imagePoint.x, imagePoint.y, -1.0);
                            cv::Mat pm = pointOfView->GetCameraParameters().GetInverseCameraMatrix() * cv::Mat(p);
                            p.x = pm.at<double>(0) / pm.at<double>(2); p.y = pm.at<double>(1) / pm.at<double>(2); p.z = 1;

                            Eigen::MatrixXd R; cv2eigen(pointOfView->GetCameraParameters().GetRotationMatrix(), R);
                            Eigen::MatrixXd T; cv2eigen(pointOfView->GetCameraParameters().GetTranslationMatrix(), T);
                            T = -1 * (R * T);

                            Eigen::MatrixXd pp(3, 1);
                            pp = R * x;
                            pp += T;

                            dx = pp(0) / pp(2) - p.x;
                            dy = pp(1) / pp(2) - p.y;

                            fvec[2 * i + 0] = dx;
                            fvec[2 * i + 1] = dy;
                        }

                        return 0;
                    }
            };

            Point::Point()
                : x(0), y(0), z(0),
                  r(0), g(0), b(0),
                  triangulated(false),
                  hidden(false)
            {
            }

            Point::Point(const Point &other)
                : x(other.x), y(other.y), z(other.z),
                  r(other.r), g(other.g), b(other.b),
                  imagePointInViews(other.imagePointInViews),
                  hidden(other.hidden),
                  triangulated(other.triangulated)
            {
            }

            Point::~Point()
            {
            }

            Point &Point::operator =(const Point &other)
            {
                x = other.x; y = other.y; z = other.z;
                r = other.r; g = other.g; b = other.b;

                imagePointInViews = other.imagePointInViews;

                hidden = other.hidden;
                triangulated = other.triangulated;

                return *this;
            }

            //bool Point::operator ==(const Point &other)
            //{
            //    return (x == other.x &&
            //            y == other.y &&
            //            z == other.z &&
            //            r == other.r &&
            //            g == other.g &&
            //            b == other.b &&
            //            triangulated == other.triangulated &&
            //            hidden == other.hidden &&
            //            imagePointInViews == other.imagePointInViews);
            //}

            Point Point::Merge(const Point &leftPoint, const Point &rightPoint)
            {
                Point mergedPoint;

                for (auto it : leftPoint.imagePointInViews)
                {
                    std::shared_ptr<PointOfView> pointOfView = it.first.lock();
                    if (pointOfView != NULL)
                    {
                        mergedPoint.AddCorrespondence(pointOfView, it.second);
                    }
                }
                for (auto it : rightPoint.imagePointInViews)
                {
                    std::shared_ptr<PointOfView> pointOfView = it.first.lock();
                    if (pointOfView != NULL)
                    {
                        mergedPoint.AddCorrespondence(pointOfView, it.second);
                    }
                }

                mergedPoint.SetColor((leftPoint.r+rightPoint.r)/2.0, (leftPoint.g+rightPoint.g)/2.0, (leftPoint.b+rightPoint.b)/2.0);

                if (leftPoint.IsTriangulated())
                {
                    mergedPoint.SetPosition(leftPoint.GetX(), leftPoint.GetY(), leftPoint.GetZ());
                    mergedPoint.SetTriangulated(leftPoint.IsTriangulated());
                }
                else if (rightPoint.IsTriangulated())
                {
                    mergedPoint.SetPosition(rightPoint.GetX(), rightPoint.GetY(), rightPoint.GetZ());
                    mergedPoint.SetTriangulated(rightPoint.IsTriangulated());
                }
                else
                {
                    // Just go with the first one, doesn't really matter at this point
                    mergedPoint.SetPosition(leftPoint.GetX(), leftPoint.GetY(), leftPoint.GetZ());
                    mergedPoint.SetTriangulated(false);
                }

                mergedPoint.SetHidden(leftPoint.IsHidden() || rightPoint.IsHidden());
                return mergedPoint;
            }

            cv::Point3d Point::GetPoint3d() const
            {
                return cv::Point3d(x, y, z);
            }

            pcl::PointXYZRGB Point::GetPCLPoint() const
            {
                pcl::PointXYZRGB point;
                point.x = (float) x; point.y = (float) y; point.z = (float) z;
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                point.rgb = *reinterpret_cast<float*>(&rgb);
                return point;
            }

            Point::operator cv::Point3d() const
            {
                return GetPoint3d();
            }

            Point::operator pcl::PointXYZRGB() const
            {
                return GetPCLPoint();
            }

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

            void Point::SetPosition(const cv::Point3d &point)
            {
                this->x = point.x;
                this->y = point.y;
                this->z = point.z;
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

            bool Point::IsHidden() const
            {
                return hidden;
            }

            void Point::SetHidden(bool hidden)
            {
                this->hidden = hidden;
            }

            int Point::GetCorrespondenceCount() const
            {
                return imagePointInViews.size();
            }

            void Point::AddCorrespondence(const std::shared_ptr<PointOfView> &pointOfView, const cv::Point2d &imagePoint)
            {
                std::weak_ptr<PointOfView> weakPointOfView(pointOfView);
                imagePointInViews[weakPointOfView] = imagePoint;
            }

            void Point::FixCorrespondenceProjection(const std::shared_ptr<PointOfView> &pointOfView, const cv::Point2d &fixedPoint)
            {
                if (HasCorrespondenceInView(pointOfView))
                {
                    std::weak_ptr<PointOfView> weakPointOfView(pointOfView);
                    imagePointInViews[weakPointOfView] = fixedPoint;
                }
            }

            bool Point::HasCorrespondenceInView(const std::shared_ptr<PointOfView> &pointOfView) const
            {
                std::weak_ptr<PointOfView> weakPointOfView(pointOfView);
                return (imagePointInViews.find(weakPointOfView) != imagePointInViews.end());
            }

            cv::Point2d Point::GetPointInView(const std::shared_ptr<PointOfView> &pointOfView) const
            {
                if (HasCorrespondenceInView(pointOfView))
                {
                    std::weak_ptr<PointOfView> weakPointOfView(pointOfView);
                    return imagePointInViews.at(weakPointOfView);
                }
                // TODO use boost.optional
                return cv::Point2d(DBL_MAX, DBL_MAX);
            }

            Point::ImagePointInViewsMap Point::GetCorrespondeces()
            {
                return imagePointInViews;
            }

            double Point::EstimateError(const std::shared_ptr<PointOfView> &pointOfView) const
            {
                double distance = -1.0;
                if (HasCorrespondenceInView(pointOfView))
                {
                    double dx, dy;
                    cv::Point2d imagePoint = GetPointInView(pointOfView);

                    cv::Point3d p(imagePoint.x, imagePoint.y, -1.0);
                    cv::Mat pm = pointOfView->GetCameraParameters().GetInverseCameraMatrix() * cv::Mat(p);
                    p.x = pm.at<double>(0) / pm.at<double>(2); p.y = pm.at<double>(1) / pm.at<double>(2); p.z = 1;

                    cv::Point2d projectedPoint = ProjectPoint(pointOfView);

                    dx = projectedPoint.x - p.x;
                    dy = projectedPoint.y - p.y;

                    distance = dx * dx + dy * dy;
                }
                return distance;
            }

            void Point::Triangulate(bool reset, bool optimize)
            {
                if (!triangulated || (triangulated && reset))
                {
                    triangulated = false;

                    if (GetUsefulPointsOfViewCount() < 2)
                    {
                        return;
                    }

                    TriangulateLinear();

                    triangulated = true;
                }

                if (triangulated && optimize)
                {
                    Refine3DPosition();
                }
            }

            void Point::Refine3DPosition()
            {
                const int m = 2 * GetUsefulPointsOfViewCount(), n = 3;
                int info;
                Eigen::VectorXd point(3);
                point(0) = x;
                point(1) = y;
                point(2) = z;

                LMDifFunctor<double> functor(n, m, this);
                Eigen::NumericalDiff<LMDifFunctor<double> > numDiff(functor);
                Eigen::LevenbergMarquardt<Eigen::NumericalDiff<LMDifFunctor<double> > > lm(numDiff);
                info = lm.minimize(point);

                x = point(0);
                y = point(1);
                z = point(2);
            }

            cv::Point2d Point::ProjectPoint(std::shared_ptr<PointOfView> pointOfView) const
            {
                Eigen::MatrixXd R; cv2eigen(pointOfView->GetCameraParameters().GetRotationMatrix(), R);
                Eigen::MatrixXd T; cv2eigen(pointOfView->GetCameraParameters().GetTranslationMatrix(), T);
                T = -1 * (R * T);

                Eigen::MatrixXd X(3, 1); X(0) = x; X(1) = y; X(2) = z;
                Eigen::MatrixXd pp(3, 1);
                pp = R * X;
                pp += T;

                return cv::Point2d(pp(0) / pp(2), pp(1) / pp(2));
            }

            std::vector<std::shared_ptr<PointOfView> > Point::GetUsefulPointsOfView() const
            {
                std::vector<std::shared_ptr<PointOfView> > usefulPointsOfView;
                std::for_each(imagePointInViews.begin(), imagePointInViews.end(), [&](const std::pair<std::weak_ptr<PointOfView>, cv::Point2d> &projection) {
                    std::shared_ptr<PointOfView> pointOfView = projection.first.lock();
                    if (pointOfView != NULL && pointOfView->GetCameraParameters().IsPoseDetermined())
                    {
                        usefulPointsOfView.push_back(pointOfView);
                    }
                });
                return usefulPointsOfView;
            }

            int Point::GetUsefulPointsOfViewCount() const
            {
                return GetUsefulPointsOfView().size();

            }

            void Point::SetTriangulated(bool triangulated)
            {
                this->triangulated = triangulated;
            }

            void Point::TriangulateLinear()
            {
                std::vector<std::shared_ptr<PointOfView> > usefulPointsOfView = GetUsefulPointsOfView();
                int usefulPointsOfViewCount = usefulPointsOfView.size();
                Eigen::MatrixXd A(2 * usefulPointsOfViewCount, 3);
                Eigen::MatrixXd B(2 * usefulPointsOfViewCount, 1);

                for (int i = 0; i < usefulPointsOfViewCount; i++)
                {
                    std::shared_ptr<PointOfView> pointOfView = usefulPointsOfView.at(i);
                    cv::Point2d imagePoint = GetPointInView(pointOfView);

                    cv::Point3d p(imagePoint.x, imagePoint.y, -1.0);
                    cv::Mat pm = pointOfView->GetCameraParameters().GetInverseCameraMatrix() * cv::Mat(p);
                    p.x = pm.at<double>(0) / pm.at<double>(2); p.y = pm.at<double>(1) / pm.at<double>(2); p.z = 1;

                    Eigen::MatrixXd R; cv2eigen(pointOfView->GetCameraParameters().GetRotationMatrix(), R);
                    Eigen::MatrixXd T; cv2eigen(pointOfView->GetCameraParameters().GetTranslationMatrix(), T);
                    T = -1 * (R * T);

                    A(2 * i + 0, 0) = R(0, 0) - p.x * R(2, 0);
                    A(2 * i + 0, 1) = R(0, 1) - p.x * R(2, 1);
                    A(2 * i + 0, 2) = R(0, 2) - p.x * R(2, 2);

                    A(2 * i + 1, 0) = R(1, 0) - p.y * R(2, 0);
                    A(2 * i + 1, 1) = R(1, 1) - p.y * R(2, 1);
                    A(2 * i + 1, 2) = R(1, 2) - p.y * R(2, 2);

                    B(2 * i + 0, 0) = p.x * T(2) - T(0);
                    B(2 * i + 1, 0) = p.y * T(2) - T(1);
                }

                Eigen::MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

                //    double error = 0.0;

                //    for (int i = 0; i < validPointsOfViewCount; i++)
                //    {
                //        error += EstimateError(validPointsOfView[i]);
                //    }

                //    std::cout << "Reprojection error for point: " << error << std::endl;

                this->x = X(0);
                this->y = X(1);
                this->z = X(2);
            }

        }
    }
}
