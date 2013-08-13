#include "Core/Object.h"

namespace Xu
{
    namespace Core
    {
        Object::Object(double confidenceLevel, std::string label)
            : confidenceLevel(confidenceLevel),
              label(label)
        {
        }

        double Object::GetConfidenceLevel() const
        {
            return confidenceLevel;
        }

        const std::string &Object::GetLabel() const
        {
            return label;
        }
    }
}
