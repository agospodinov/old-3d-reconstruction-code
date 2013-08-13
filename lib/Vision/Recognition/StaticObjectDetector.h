#ifndef STATICOBJECTDETECTOR_H
#define STATICOBJECTDETECTOR_H

#include "IObjectDetector.h"

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            class StaticObjectDetector : public IObjectDetector
            {
                public:
                    StaticObjectDetector(int spatialRadius = 50, int colorRadius = 15, int minSize = 30);
                    virtual ~StaticObjectDetector();

                    virtual std::vector<Xu::Core::Object> Detect(Vision::Core::VisualData data);

                private:
                    const int spatialRadius;
                    const int colorRadius;
                    const int minSize;

            };
        }
    }
}

#endif // STATICOBJECTDETECTOR_H
