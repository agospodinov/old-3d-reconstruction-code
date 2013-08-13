#ifndef SHAPEBASEDOBJECTDETECTOR_H
#define SHAPEBASEDOBJECTDETECTOR_H

#include "IObjectDetector.h"

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            class ShapeBasedObjectDetector : public IObjectDetector
            {
                public:
                    ShapeBasedObjectDetector();
                    virtual ~ShapeBasedObjectDetector();

                    virtual std::vector<Xu::Core::Object> Detect(Vision::Core::VisualData data);
            };
        }
    }
}

#endif // SHAPEBASEDOBJECTDETECTOR_H
