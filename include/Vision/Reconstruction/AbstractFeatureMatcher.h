#ifndef ABSTRACTFEATUREMATCHER_H
#define ABSTRACTFEATUREMATCHER_H

#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>

#include "IImageMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Feature;
        }

        namespace Reconstruction
        {
            class AbstractFeatureMatcher : public IImageMatcher
            {
                public:
                    AbstractFeatureMatcher(int matchNLast, int keepMLastOnGPU);
                    virtual ~AbstractFeatureMatcher();

                    virtual void AddImage(std::shared_ptr<Core::PointOfView> &pointOfView);

                protected:
                    struct Match
                    {
                        Match(uint leftFeatureIndex, uint rightFeatureIndex, const cv::Point2d &leftPoint, const cv::Point2d &rightPoint)
                            : leftFeatureIndex(leftFeatureIndex),
                              rightFeatureIndex(rightFeatureIndex),
                              leftPoint(leftPoint),
                              rightPoint(rightPoint)
                        {
                        }

                        uint leftFeatureIndex;
                        uint rightFeatureIndex;
                        cv::Point2d leftPoint;
                        cv::Point2d rightPoint;
                    };
                    typedef std::vector<AbstractFeatureMatcher::Match> MatchList;

                    virtual void DetectAlgorithmSpecificFeatures(int pointOfViewIndex) = 0;
                    virtual MatchList MatchAlgorithmSpecificFeatures(int leftPOVIndex, int rightPOVIndex) = 0;

                    inline std::shared_ptr<Core::PointOfView> &GetPointOfView(int index)
                    {
                        return pointsOfView.at(index % (matchNLast + 1));
                    }

                    inline int GetCurrentImageIndex() const
                    {
                        return processedImageCount - 1;
                    }

                    inline int GetNthLastImageIndex() const
                    {
                        return std::max(GetCurrentImageIndex() - matchNLast, 0);
                    }

                    inline int GetKeepMLastOnGPU() const
                    {
                        return keepMLastOnGPU;
                    }

                private:
                    void CreateAndMatchFeatures();
                    std::shared_ptr<Core::Feature> CreateFeature(const std::shared_ptr<Core::PointOfView> &leftPOV,
                                                           const std::shared_ptr<Core::PointOfView> &rightPOV,
                                                           const Match &match);

                    void CorrectMatches(std::vector<Match> &matches,
                                        const std::shared_ptr<Core::PointOfView> &leftPOV,
                                        const std::shared_ptr<Core::PointOfView> &rightPOV) const;

                    /**
                     * @brief The last N points of view.
                     *
                     * The index of the n'th last point of view is
                     * nthLastImageIndex % matchNLast and the current one is
                     * currentImageIndex % matchNLast.
                     */
                    std::vector<std::shared_ptr<Core::PointOfView> > pointsOfView;

                    /**
                     * @brief The number of images processed.
                     */
                    int processedImageCount;

                    /**
                     * @brief The size of the subset of descriptors to match.
                     */
                    int matchNLast;

                    /**
                     * @brief The number of keypoints and descriptors to keep in GPU memory
                     * before moving them to the host memory.
                     */
                    int keepMLastOnGPU;

                    int gpuLastDownloadIteration;

            };

        }
    }
}

#endif // ABSTRACTFEATUREMATCHER_H
