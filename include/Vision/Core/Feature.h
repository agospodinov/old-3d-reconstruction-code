#ifndef FEATURE_H
#define FEATURE_H

#include "Point.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Feature : public Point
            {
                public:
                    typedef std::shared_ptr<Feature> Ptr;
                    typedef std::vector<Feature::Ptr> PtrList;

                    Feature();
                    Feature(const Feature &other);
                    virtual ~Feature();

                    Feature &operator=(const Feature &other);
                    //    bool operator==(const Feature &other);

                    static Feature Merge(const Feature &leftFeature, const Feature &rightFeature);

                    void AddCorrespondence(const std::shared_ptr<PointOfView> &pointOfView, int pointIndex, cv::Point2d point);

                    int GetPointIndexInView(const std::shared_ptr<PointOfView> &pointOfView) const;

                private:
                    std::map<std::weak_ptr<PointOfView>, int, std::owner_less<std::weak_ptr<PointOfView> > > imagePointIndexInViews;

            };
        }
    }
}

#endif // FEATURE_H
