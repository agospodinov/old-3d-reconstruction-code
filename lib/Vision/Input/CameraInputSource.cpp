#include "Vision/Input/CameraInputSource.h"

#include "Vision/Core/SingleViewImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Input
        {
            CameraInputSource::CameraInputSource()
                : capture(0)
            {
            }

            CameraInputSource::CameraInputSource(const cv::VideoCapture &videoCapture)
                : capture(videoCapture)
            {
                if (IsNextFrameAvailable())
                {
                    capture.retrieve(nextFrame);
                    frameSize = nextFrame.size();
                }

            }

            CameraInputSource::~CameraInputSource()
            {
            }

            std::unique_ptr<Core::IImage> CameraInputSource::GetNextFrame()
            {
                cv::Mat currentFrame = nextFrame;
                if (IsNextFrameAvailable())
                {
                    capture.retrieve(nextFrame);
                    // I think we can safely assume video has a fixed width and height.
                }
                return std::unique_ptr<Core::IImage>(new Core::SingleViewImage(currentFrame));
            }

            bool CameraInputSource::IsNextFrameAvailable()
            {
                return capture.grab();
            }

            cv::Size CameraInputSource::GetSize() const
            {
                return frameSize;
            }
        }
    }
}
