#ifndef POINT_H
#define POINT_H

#include <memory>
#include <vector>

#include <boost/optional.hpp>
#include <boost/multi_index_container_fwd.hpp>
#include <boost/multi_index/sequenced_index_fwd.hpp>
#include <boost/multi_index/hashed_index_fwd.hpp>
#include <boost/multi_index/member.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

//#include <pcl/common/common.h>

#include "Vision/Core/Projection.h"

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
                    Point();
                    virtual ~Point();

                    /// delete or refactor
//                    pcl::PointXYZRGB GetPCLPoint() const;
                    /// end delete or refactor

                    double GetX() const;
                    double GetY() const;
                    double GetZ() const;

                    void SetPosition(double x, double y, double z);

                    uchar GetR() const;
                    uchar GetG() const;
                    uchar GetB() const;

                    void SetColor(uchar r, uchar g, uchar b);

                    bool IsTriangulated() const;
                    void SetTriangulated(bool triangulated)
                    {
                        this->triangulated = triangulated;
                    }

                    const std::vector<Projection> &GetProjections() const;
                    const boost::optional<Projection> GetProjection(const std::shared_ptr<PointOfView> &pointOfView) const;
                    bool HasProjection(const std::shared_ptr<PointOfView> &pointOfView) const;
                    void AddProjection(const Projection &projection);
                    void RemoveProjection(const std::shared_ptr<PointOfView> &pointOfView);

                    void Triangulate(bool reset = false, bool optimize = true);

//                    boost::optional<double> EstimateError(const std::shared_ptr<PointOfView> &pointOfView) const;
//                    Projection ProjectPoint(const std::shared_ptr<PointOfView> &pointOfView) const;

                private:
                    friend bool operator ==(const Point &left, const Point &right);
                    friend bool operator <(const Point &left, const Point &right);

                    void TriangulateLinear(const std::vector<Projection> &usefulProjections);
                    void Refine3DPosition(const std::vector<Projection> &usefulProjections);

                    double x, y, z;
                    uchar r, g, b;

                    bool triangulated;

                    std::vector<Projection> projections;

                    // TODO
//                    using namespace boost::multi_index;
//                    typedef boost::multi_index_container<
//                        Projection,
//                        indexed_by<
//                            sequenced<>,
//                            hashed_unique<member<Projection, std::weak_ptr<PointOfView>, &Projection::pointOfView> >
//                        >
//                    > ProjectionSet;

//                    ProjectionSet projections;
            };

        }
    }
}

#endif // POINT_H
