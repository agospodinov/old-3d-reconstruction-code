#include "Projection.h"

#include "Vision/Core/PointOfView.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            Projection::Projection(double x, double y, const std::shared_ptr<PointOfView> &pointOfView, bool actualProjection)
                : Projection(x, y, pointOfView, nullptr, actualProjection)
            {
            }

            Projection::Projection(double x, double y, const std::shared_ptr<PointOfView> &pointOfView, Point *associatedPoint, bool actualProjection)
                : x(x),
                  y(y),
                  pointOfView(pointOfView),
                  associatedPoint(associatedPoint),
                  actualProjection(actualProjection)
            {

            }

            double Projection::GetX() const
            {
                return x;
            }

            void Projection::SetX(double x)
            {
                this->x = x;
            }

            double Projection::GetY() const
            {
                return y;
            }

            void Projection::SetY(double y)
            {
                this->y = y;
            }
            
            std::shared_ptr<PointOfView> Projection::GetPointOfView() const
            {
                return pointOfView.lock();
            }

            Point *Projection::GetAssociatedPoint() const
            {
                return associatedPoint;
            }

            void Projection::SetAssociatedPoint(Point * const point)
            {
                this->associatedPoint = point;
            }
            
            bool Projection::IsActualProjection() const
            {
                return actualProjection;
            }

            bool operator ==(const Projection &left, const Projection &right)
            {
                return (left.x == right.x && left.y == right.y
                        && left.associatedPoint == right.associatedPoint
                        && left.pointOfView.lock().get() == right.pointOfView.lock().get()
                        && left.actualProjection == right.actualProjection);
            }

            bool operator <(const Projection &left, const Projection &right)
            {
                return std::tie(left.x, left.y,
                                left.actualProjection,
                                left.associatedPoint) <
                        std::tie(right.x, right.y,
                                 right.actualProjection,
                                 right.associatedPoint)
                        || left.pointOfView.owner_before(right.pointOfView);
            }
        }
    }
}
