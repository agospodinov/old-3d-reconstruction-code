#include "Core/Object.h"

#include "Core/ObjectData.h"

namespace Xu
{
    namespace Core
    {
        Object::Object(const std::shared_ptr<ObjectData> &data, double confidenceLevel, std::string label)
            : confidenceLevel(confidenceLevel),
              label(label)
        {
            objectData.push_back(data);
        }

        Object::Object(const std::vector<std::shared_ptr<ObjectData> > &data, double confidenceLevel, std::string label)
            : objectData(data),
              confidenceLevel(confidenceLevel),
              label(label)
        {
        }

        const std::vector<std::shared_ptr<ObjectData> > &Object::GetData() const
        {
            return objectData;
        }

        void Object::AddData(const std::shared_ptr<ObjectData> &data)
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
