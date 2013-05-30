#ifndef OBJECT_H
#define OBJECT_H

#include <vector>
#include <string>

#include "Vision/Core/VisualData.h"

namespace Xu
{
    namespace Core
    {
        class ObjectData;

        class Object
        {
            public:
                Object(const Vision::Core::VisualData &data, double confidenceLevel = 0.5, std::string label = "");
                Object(const std::vector<Vision::Core::VisualData> &data, double confidenceLevel = 0.5, std::string label = "");

                const std::vector<Vision::Core::VisualData> &GetData() const;
                void AddData(const Vision::Core::VisualData &data);

                double GetConfidenceLevel() const;
                const std::string &GetLabel() const;

            private:
                std::string label;

                double confidenceLevel;

                std::vector<Vision::Core::VisualData> objectData;
        };
    }
}

#endif // OBJECT_H
