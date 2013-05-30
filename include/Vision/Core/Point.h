#ifndef POINT_H
#define POINT_H

#include <map>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/shared_ptr.hpp>

#include <pcl/common/common.h>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {

            class PointOfView;

            class Point
            {
                public:

                    typedef std::map<std::weak_ptr<PointOfView>, cv::Point2d, std::owner_less<std::weak_ptr<PointOfView> > > ImagePointInViewsMap;

                    /* // TODO Implement properly if there's enough time
                    class Projection
                    {
                    public:
                        double GetX() const;
                        double GetY() const;

                        std::weak_ptr<PointOfView> GetPointOfView() const;

                        bool IsActualProjection() const;
                    private:
                        std::weak_ptr<PointOfView> pointOfView;
                        double x, y;

                        bool actualProjection;

                        friend class Point;
                    };
                    */

                    Point();
                    Point(const Point &);
                    virtual ~Point();

                    Point &operator=(const Point &other);
//                    bool operator==(const Point &other);

                    static Point Merge(const Point &leftPoint, const Point &rightPoint);

                    cv::Point3d GetPoint3d() const;
                    pcl::PointXYZRGB GetPCLPoint() const;

                    operator cv::Point3d() const;
                    operator pcl::PointXYZRGB() const;

                    double GetX() const;
                    double GetY() const;
                    double GetZ() const;

                    void SetPosition(double x, double y, double z);
                    void SetPosition(const cv::Point3d &point);

                    uchar GetR() const;
                    uchar GetG() const;
                    uchar GetB() const;

                    void SetColor(uchar r, uchar g, uchar b);

                    bool IsTriangulated() const;
                    void SetTriangulated(bool);

                    bool IsHidden() const;
                    void SetHidden(bool);

                    int GetCorrespondenceCount() const;

                    void AddCorrespondence(const std::shared_ptr<PointOfView> &pointOfView, const cv::Point2d &imagePoint);
                    void FixCorrespondenceProjection(const std::shared_ptr<PointOfView> &pointOfView, const cv::Point2d &fixedPoint);

                    bool HasCorrespondenceInView(const std::shared_ptr<PointOfView> &pointOfView) const;
                    cv::Point2d GetPointInView(const std::shared_ptr<PointOfView> &pointOfView) const;

                    ImagePointInViewsMap GetCorrespondeces();

                    std::vector<std::shared_ptr<PointOfView> > GetUsefulPointsOfView() const;
                    int GetUsefulPointsOfViewCount() const;

                    double EstimateError(const std::shared_ptr<PointOfView> &pointOfView) const;
                    cv::Point2d ProjectPoint(std::shared_ptr<PointOfView> pointOfView) const;

                    void Triangulate(bool reset = false, bool optimize = true);

                protected:
                    void Refine3DPosition();

                    ImagePointInViewsMap imagePointInViews;

                private:
                    void TriangulateLinear();

                    double x, y, z;
                    uchar r, g, b;

                    bool triangulated;
                    bool hidden;
            };

        }
    }
}

#endif // POINT_H
