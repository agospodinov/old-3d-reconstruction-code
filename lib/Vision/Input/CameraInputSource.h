#ifndef CAMERAINPUTSOURCE_H
#define CAMERAINPUTSOURCE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "IInputSource.h"

namespace Xu
{
    namespace Vision
    {
        namespace Input
        {
            class CameraInputSource : public IInputSource
            {
                public:
                    CameraInputSource();
                    CameraInputSource(const cv::VideoCapture &);
                    virtual ~CameraInputSource();

                    virtual std::unique_ptr<Core::IImage> GetNextFrame();
                    virtual bool IsNextFrameAvailable();
                    virtual cv::Size GetSize() const;

                private:
                    cv::VideoCapture capture;

                    cv::Mat nextFrame;
                    cv::Size frameSize;
            };
        }
    }
}

#endif // CAMERAINPUTSOURCE_H
