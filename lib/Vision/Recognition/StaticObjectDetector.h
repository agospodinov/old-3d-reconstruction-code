#ifndef STATICOBJECTDETECTOR_H
#define STATICOBJECTDETECTOR_H

//#include "Vision/Core/PointOfView.h"
#include "Core/Data.h"
#include "Vision/Recognition/IObjectDetector.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class PointOfView;
        }

        namespace Recognition
        {
            class StaticObjectDetector : public IObjectDetector<Vision::Core::PointOfView>
            {
                public:
                    StaticObjectDetector(int spatialRadius = 50, int colorRadius = 15, int minSize = 30);
                    virtual ~StaticObjectDetector();

                    virtual std::vector<Xu::Core::Object> Detect(Xu::Core::Data<Vision::Core::PointOfView> data);

                private:
                    const int spatialRadius;
                    const int colorRadius;
                    const int minSize;

            };
        }
    }
}

#endif // STATICOBJECTDETECTOR_H
