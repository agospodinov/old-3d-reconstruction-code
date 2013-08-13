#ifndef IMAGELISTINPUTSOURCE_H
#define IMAGELISTINPUTSOURCE_H

#include <string>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "IInputSource.h"
#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Input
        {

            class ImageListInputSource : public IInputSource
            {
                public:
                    ImageListInputSource(std::list<std::string>);
                    ImageListInputSource(std::string);
                    virtual ~ImageListInputSource();

                    virtual std::unique_ptr<Core::IImage> GetNextFrame();
                    virtual bool IsNextFrameAvailable();
                    virtual cv::Size GetSize() const;

                private:
                    std::list<std::string> imageList;

                    cv::Mat nextFrame;
                    cv::Size frameSize;
            };

        }
    }
}

#endif // IMAGELISTINPUTSOURCE_H
