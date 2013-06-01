#ifndef OBJECT_H
#define OBJECT_H

#include <memory>
#include <string>
#include <vector>

namespace Xu
{
    namespace Core
    {
        class ObjectData;

        class Object
        {
            public:
                Object(const std::shared_ptr<ObjectData> &data, double confidenceLevel = 0.5, std::string label = "");
                Object(const std::vector<std::shared_ptr<ObjectData> > &data, double confidenceLevel = 0.5, std::string label = "");

                const std::vector<std::shared_ptr<ObjectData> > &GetData() const;
                void AddData(const std::shared_ptr<ObjectData> &data);

                double GetConfidenceLevel() const;
                const std::string &GetLabel() const;

            private:
                std::string label;

                double confidenceLevel;

                std::vector<std::shared_ptr<ObjectData> > objectData;
        };
    }
}

#endif // OBJECT_H
