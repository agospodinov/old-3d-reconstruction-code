#ifndef IOBJECTDETECTOR_H
#define IOBJECTDETECTOR_H

#include <vector>

#include "Core/Object.h"
#include "Vision/Core/VisualData.h"

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            class IObjectDetector
            {
                public:
                    virtual ~IObjectDetector();

                    virtual std::vector<Xu::Core::Object> Detect(Vision::Core::VisualData data) = 0;
            };
        }
    }
}

#endif // IOBJECTDETECTOR_H
