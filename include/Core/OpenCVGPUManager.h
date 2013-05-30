#ifndef OPENCVGPUMANAGER_H
#define OPENCVGPUMANAGER_H

#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpumat.hpp>

// TODO find a better place for this class... I don't think the Core module is
// the best place for it...
class OpenCVGPUManager
{
public:
    OpenCVGPUManager();

    cv::gpu::GpuMat &Allocate(cv::Mat cpuMat);


private:
    /**
     * @brief Move Moves the oldest matrix from GPU to CPU.
     */
    void Move();

    /**
     * @brief cpuAllocatedMatrices Matrices that have been moved to the CPU.
     */
    std::deque<cv::Mat> cpuAllocatedMatrices;

    /**
     * @brief gpuAllocatedMatrices Matrices that are currently on the GPU.
     */
    std::deque<cv::gpu::GpuMat> gpuAllocatedMatrices;
};

#endif // OPENCVGPUMANAGER_H
