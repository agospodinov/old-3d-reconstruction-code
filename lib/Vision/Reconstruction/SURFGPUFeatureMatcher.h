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
                    SURFGPUFeatureMatcher(const std::shared_ptr<Core::Scene> &scene, int matchNLast = 4, int keepMLastOnGPU = 5);
                    virtual ~SURFGPUFeatureMatcher();

                    virtual std::vector<Core::Projection> DetectAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &pointOfView);
                    virtual std::vector<Match> MatchAlgorithmSpecificFeatures(const std::shared_ptr<Core::PointOfView> &leftPOV, const std::shared_ptr<Core::PointOfView> &rightPOV);

                private:
                    cv::gpu::SURF_GPU featureExtractor;
                    cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > matcher;

                    /**
                     * Allowing descriptors to live on the GPU greatly speeds up the process as we
                     * don't have to download to/upload from host memory from/to GPU memory
                     * every time we want to do something. However, GPU memory is usually small,
                     * so we can only keep a limited amount of them on the GPU.
                     */
                    boost::circular_buffer<std::pair<std::shared_ptr<Core::PointOfView>, cv::gpu::GpuMat> > imageDescriptorsGPU;
            };
        }
    }
}

#endif // SURFFEATUREMATCHER_H
