#ifndef MOVINGOBJECTDETECTOR_H
#define MOVINGOBJECTDETECTOR_H

#include "IObjectDetector.h"

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            class MovingObjectDetector : public IObjectDetector
            {
                public:
                    MovingObjectDetector();
                    virtual ~MovingObjectDetector();

                    virtual std::vector<Xu::Core::Object> Detect(Vision::Core::VisualData data);

                private:

            };
        }
    }
}
#endif // MOVINGOBJECTDETECTOR_H
