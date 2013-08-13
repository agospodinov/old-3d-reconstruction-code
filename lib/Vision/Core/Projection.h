#ifndef PROJECTION_H
#define PROJECTION_H

#include <memory>

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Point;
            class PointOfView;

            class Projection
            {
                public:
                    Projection(double x, double y, const std::shared_ptr<PointOfView> &pointOfView, bool actualProjection = false);

                    double GetX() const;
                    void SetX(double x);

                    double GetY() const;
                    void SetY(double y);

                    std::shared_ptr<PointOfView> GetPointOfView() const;
                    Point *GetAssociatedPoint() const;
                    void SetAssociatedPoint(Point * const point);

                    bool IsActualProjection() const;
                private:
                    friend bool operator ==(const Projection &left, const Projection &right);
                    friend bool operator <(const Projection &left, const Projection &right);

                    double x, y;
                    std::weak_ptr<PointOfView> pointOfView;
                    Point *associatedPoint;
                    bool actualProjection;

                    friend class Point;
            };
        }
    }
}

#endif // PROJECTION_H
