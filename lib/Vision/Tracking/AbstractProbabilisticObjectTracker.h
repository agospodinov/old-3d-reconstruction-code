#ifndef ABSTRACTPROBABILISTICOBJECTTRACKER_H
#define ABSTRACTPROBABILISTICOBJECTTRACKER_H

#include "Vision/Tracking/IObjectTracker.h"

namespace Xu
{
    namespace Vision
    {
        namespace Tracking
        {
            class AbstractProbabilisticObjectTracker : public IObjectTracker
            {
                public:
                    AbstractProbabilisticObjectTracker();
                    virtual ~AbstractProbabilisticObjectTracker();
            };
        }
    }
}

#endif // ABSTRACTPROBABILISTICOBJECTTRACKER_H
