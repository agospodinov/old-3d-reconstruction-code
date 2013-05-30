#include "Core/Object.h"

#include "Core/ObjectData.h"

namespace Xu
{
    namespace Core
    {
        Object::Object(const Vision::Core::VisualData &data, double confidenceLevel, std::string label)
            : confidenceLevel(confidenceLevel),
              label(label)
        {
            objectData.push_back(data);
        }

        Object::Object(const std::vector<Vision::Core::VisualData> &data, double confidenceLevel, std::string label)
            : objectData(data),
              confidenceLevel(confidenceLevel),
              label(label)
        {
        }

        const std::vector<Vision::Core::VisualData> &Object::GetData() const
        {
            return objectData;
        }

        void Object::AddData(const Vision::Core::VisualData &data)
        {
            objectData.push_back(data);
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
