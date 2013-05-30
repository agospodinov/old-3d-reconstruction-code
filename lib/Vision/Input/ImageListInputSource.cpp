#include "Vision/Input/ImageListInputSource.h"

#include <iostream>

#include "Vision/Core/SingleViewImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Input
        {
            ImageListInputSource::ImageListInputSource(std::list<std::string> images)
                : imageList(images)
            {
                if (IsNextFrameAvailable())
                {
                    std::string nextImage = imageList.front();
                    imageList.pop_front();
                    std::cout << "Processing image: " << nextImage << std::endl;
                    nextFrame = cv::imread(nextImage);
                    frameSize = nextFrame.size();
                }
            }

            ImageListInputSource::ImageListInputSource(std::string fileName)
            {
            }

            ImageListInputSource::~ImageListInputSource()
            {
            }

            std::unique_ptr<Core::IImage> ImageListInputSource::GetNextFrame()
            {
                cv::Mat currentFrame = nextFrame;
                if (IsNextFrameAvailable())
                {
                    std::string nextImage = imageList.front();
                    imageList.pop_front();
                    std::cout << "Processing image: " << nextImage << std::endl;
                    nextFrame = cv::imread(nextImage);
                    frameSize = nextFrame.size();
                }
                return std::unique_ptr<Core::IImage>(new Core::SingleViewImage(currentFrame));
            }

            bool ImageListInputSource::IsNextFrameAvailable()
            {
                return !imageList.empty();
            }

            cv::Size ImageListInputSource::GetSize() const
            {
                return frameSize;
            }

        }
    }
}
