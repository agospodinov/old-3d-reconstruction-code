#include "Vision/Recognition/CompositeObjectDetector.h"

#include <map>

namespace Xu
{
    namespace Vision
    {
        namespace Recognition
        {
            CompositeObjectDetector::CompositeObjectDetector()
            {
            }

            CompositeObjectDetector::~CompositeObjectDetector()
            {
            }

            std::vector<Xu::Core::Object> CompositeObjectDetector::Detect(Core::VisualData data)
            {
//                std::map<const IObjectDetector *, std::vector<Xu::Core::Object> > objects;
//                for (const IObjectDetector &detector : objectDetectors)
//                {
//                    objects[&detector] = detector.Detect(data);
//                }

                // Merge results...
            }

        }
    }
}
