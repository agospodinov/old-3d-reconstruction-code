#ifndef ABSTRACTDENSEMATCHER_H
#define ABSTRACTDENSEMATCHER_H

#include <vector>

#include <opencv2/core/core.hpp>

#include "IImageMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {
            class AbstractDenseMatcher : public IImageMatcher
            {
                public:
                    AbstractDenseMatcher(std::size_t bundleSize = 4, std::size_t iterations = 3);

                    virtual void ProcessImage(std::shared_ptr<Core::PointOfView> &pointOfView);

                protected:
                    virtual cv::Mat GetFlowMatrix(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> rightPOV) = 0;

                private:
                    std::shared_ptr<Core::PointOfView> referenceView;
                    std::vector<std::shared_ptr<Core::PointOfView> > comparisonViews;

                    std::size_t iterations;
            };
        }
    }
}

#endif // ABSTRACTDENSEMATCHER_H
