#ifndef IINPUTSOURCE_H
#define IINPUTSOURCE_H

#include <memory>

#include <opencv2/core/core.hpp>

#include "Vision/Core/IImage.h"

namespace Xu
{
    namespace Vision
    {
        namespace Input
        {

            class IInputSource
            {
                public:
                    virtual ~IInputSource();

                    virtual std::unique_ptr<Core::IImage> GetNextFrame() = 0;
                    virtual bool IsNextFrameAvailable() = 0;
                    virtual cv::Size GetSize() const = 0;
            };

        }
    }
}

#endif // IINPUTSOURCE_H
