#ifndef SURFFEATUREMATCHER_H
#define SURFFEATUREMATCHER_H

#include <map>

#include <boost/circular_buffer.hpp>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/gpu.hpp>

#include "AbstractFeatureMatcher.h"

namespace Xu
{
    namespace Vision
    {
        namespace Reconstruction
        {

            class SURFGPUFeatureMatcher : public AbstractFeatureMatcher
            {
                public:
                    SURFGPUFeatureMatcher(Core::Scene &scene, int matchNLast = 4, int keepMLastOnGPU = 10);
                    virtual ~SURFGPUFeatureMatcher();

                    virtual void DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView);
                    virtual std::vector<Match> MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV);

                private:
                    cv::gpu::SURF_GPU featureExtractor;

                    /**
                     * Allowing keypoints to live on the GPU greatly speeds up the process as we
                     * don't have to download/upload to/from host memory from/to GPU memory
                     * every time we want to do something. However, GPU memory is usually small,
                     * so we need to download them every X iterations.
                     *
                     * @see keepMLastOnGPU
                     * @see gpuLastDownloadIteration
                     */
                    boost::circular_buffer<std::pair<std::shared_ptr<Core::PointOfView>, cv::gpu::GpuMat> > imageKeypointsGPU;

                    /**
                     * The same concept as imageKeypointsGPU applies here too.
                     *
                     * @see imageKeypointsGPU
                     */
                    boost::circular_buffer<std::pair<std::shared_ptr<Core::PointOfView>, cv::gpu::GpuMat> > imageDescriptorsGPU;

            };
        }
    }
}

#endif // SURFFEATUREMATCHER_H
