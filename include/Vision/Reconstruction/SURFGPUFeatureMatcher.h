#ifndef SURFFEATUREMATCHER_H
#define SURFFEATUREMATCHER_H

#include <vector>
#include <map>

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
                    SURFGPUFeatureMatcher(int matchNLast = 4, int keepMLastOnGPU = 10);
                    virtual ~SURFGPUFeatureMatcher();

                    virtual void DetectAlgorithmSpecificFeatures(int pointOfViewIndex);
                    virtual MatchList MatchAlgorithmSpecificFeatures(int leftPOVIndex, int rightPOVIndex);

                protected:

                    //    void Matchx(int i, int j);
                    //    void Download();

                private:
                    cv::gpu::SURF_GPU featureExtractor;

                    //    vector<boost::shared_ptr<vector<KeyPoint> > > imageKeypoints;
                    //    vector<boost::shared_ptr<vector<float> > > imageDescriptors;
                    //    std::vector<std::shared_ptr<PointsToTrack> > pointsToTrack;

                    /**
                     * Allowing keypoints to live on the GPU greatly speeds up the process as we
                     * don't have to download/upload to/from host memory from/to GPU memory
                     * every time we want to do something. However, GPU memory is usually small,
                     * so we need to download them every X iterations.
                     *
                     * @see keepMLastOnGPU
                     * @see gpuLastDownloadIteration
                     */
                    std::vector<cv::gpu::GpuMat> imageKeypointsGPU;

                    /**
                     * The same concept as imageKeypointsGPU applies here too.
                     *
                     * @see imageKeypointsGPU
                     */
                    std::vector<cv::gpu::GpuMat> imageDescriptorsGPU;

                    /**
                     * The key is a pair of indices. These indices, call them i and j, represent
                     * the pair of descriptors, between which the matches (the value of the map)
                     * are.
                     *
                     * Given the vector imageDescriptors (imgDscVec is a mock name for a vector
                     * of descriptors, since descriptors are stored in a vector<float>):
                     *
                     * { imgDscVec0, imgDscVec1, imgDscVec2, imgDscVec3 }
                     *
                     * then the imageMatches map would be the following (simplified for
                     * readability; imageMatches<i><j> means these are the matches between
                     * imgDscVec<i> and imgDscVec<j>):
                     *
                     * { pair(0, 0)=imgMatches00, pair(0, 1)=imgMatches01,
                     *   pair(0, 2)=imgMatches02, pair(0, 3)=imgMatches03,
                     *   ...
                     *   pair(3, 2)=imgMatches32, pair(3, 3)=imgMatches33 }
                     *
                     * Note that given N = matchNLast only matches between the last (N*(N-1))/2
                     * pairs are kept. This means if we're well ahead in the video, we don't
                     * want to try to match the current frame with one of the first frames, so
                     * instead we only match each pair from the subset of the last N
                     * imageDescriptors.
                     *
                     * @see matchNLast
                     */
                    std::map<std::pair<int, int>, std::vector<AbstractFeatureMatcher::Match> > imageMatches;

            };
        }
    }
}

#endif // SURFFEATUREMATCHER_H
