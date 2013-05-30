#include "Vision/Core/Feature.h"

#include "Vision/Core/PointOfView.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            Feature::Feature()
            {
            }

            Feature::Feature(const Feature &other)
                : Point(other),
                  imagePointIndexInViews(other.imagePointIndexInViews)
            {
            }

            Feature::~Feature()
            {
            }

            Feature &Feature::operator =(const Feature &other)
            {
                Point::operator =(other);
                imagePointIndexInViews = other.imagePointIndexInViews;

                return *this;
            }

            //bool Feature::operator ==(const Feature &other)
            //{
            //    return (Point::operator ==(other) &&
            //            imagePointIndexInViews == other.imagePointIndexInViews);
            //}

            Feature Feature::Merge(const Feature &leftFeature, const Feature &rightFeature)
            {
                Feature mergedFeature;

                mergedFeature.imagePointIndexInViews.insert(leftFeature.imagePointIndexInViews.begin(), leftFeature.imagePointIndexInViews.end());
                mergedFeature.imagePointIndexInViews.insert(rightFeature.imagePointIndexInViews.begin(), rightFeature.imagePointIndexInViews.end());

                mergedFeature.imagePointInViews.insert(leftFeature.imagePointInViews.begin(), leftFeature.imagePointInViews.end());
                mergedFeature.imagePointInViews.insert(rightFeature.imagePointInViews.begin(), rightFeature.imagePointInViews.end());

                // Maybe not the most optimal way to do this but I doubt
                // it's going to get to the point of me needing to
                // optimize this method.
                Point mergedPoint = Point::Merge(leftFeature, rightFeature);

                mergedFeature.SetPosition(mergedPoint.GetX(), mergedPoint.GetY(), mergedPoint.GetZ());
                mergedFeature.SetColor(mergedPoint.GetR(), mergedPoint.GetG(), mergedPoint.GetB());
                mergedFeature.SetTriangulated(mergedPoint.IsTriangulated());
                mergedFeature.SetHidden(mergedPoint.IsHidden());

                return mergedFeature;
            }

            void Feature::AddCorrespondence(const std::shared_ptr<PointOfView> &pointOfView, int pointIndex, cv::Point2d point)
            {
                std::weak_ptr<PointOfView> weakPointOfView(pointOfView);
                imagePointIndexInViews[weakPointOfView] = pointIndex;
                IImage::Size imageSize = pointOfView->GetImage()->GetSize();
                Point::AddCorrespondence(pointOfView, cv::Point2d(point.x - 0.5 * imageSize.width, point.y - 0.5 * imageSize.height));
            }

            int Feature::GetPointIndexInView(const std::shared_ptr<PointOfView> &pointOfView) const
            {
                if (HasCorrespondenceInView(pointOfView))
                {
                    std::weak_ptr<PointOfView> weakPointOfView(pointOfView);
                    return imagePointIndexInViews.at(weakPointOfView);
                }
                // TODO use boost.optional
                return -1;
            }


        }
    }
}
