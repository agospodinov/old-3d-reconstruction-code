#ifndef SHAPEBASEDOBJECTDETECTOR_H
#define SHAPEBASEDOBJECTDETECTOR_H

#include "IObjectDetector.h"

namespace Xu
{
    namespace Vision
    {
        namespace Core
        {
            class Scene;
        }

        namespace Recognition
        {
            class ShapeBasedObjectDetector : public IObjectDetector<std::shared_ptr<Vision::Core::Scene> >
            {
                public:
                    ShapeBasedObjectDetector();
                    virtual ~ShapeBasedObjectDetector();

                    virtual std::vector<Xu::Core::Object> Detect(Xu::Core::Data<std::shared_ptr<Vision::Core::Scene> > data);
            };
        }
    }
}

#endif // SHAPEBASEDOBJECTDETECTOR_H
