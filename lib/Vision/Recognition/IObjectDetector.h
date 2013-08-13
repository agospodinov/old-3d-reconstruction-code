#ifndef IOBJECTDETECTOR_H
#define IOBJECTDETECTOR_H

#include <vector>

#include "Core/Object.h"
#include "Core/Data.h"

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            template <typename T>
            class IObjectDetector
            {
                public:
                    IObjectDetector()
                    {
                    }

                    virtual ~IObjectDetector()
                    {
                    }

                    virtual std::vector<Xu::Core::Object> Detect(Xu::Core::Data<T> data) = 0;
            };
        }
    }
}

#endif // IOBJECTDETECTOR_H
