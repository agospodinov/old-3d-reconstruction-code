#ifndef OBJECT_H
#define OBJECT_H

#include <memory>
#include <string>
#include <vector>

#include <boost/any.hpp>

#include "Core/Data.h"

namespace Xu
{
    namespace Core
    {
        class Object
        {
            public:
                Object(double confidenceLevel = 0.5, std::string label = "");

                const std::vector<Data<boost::any> > &GetData() const
                {
                    return objectData;
                }

                template <typename T>
                std::vector<Data<T> > GetData() const
                {
                    std::vector<Data<T> > dataForType;

                    for (Data<boost::any> data : objectData)
                    {
                        if (data.GetItem().type() == typeid(T))
                        {
                            dataForType.push_back(boost::any_cast<T>(data.GetItem()));
                        }
                    }

                    return dataForType;
                }

                template <typename T>
                void AddData(Data<T> data)
                {
                    objectData.push_back(Data<boost::any>(std::move(data.GetItem())));
                }

                double GetConfidenceLevel() const;
                const std::string &GetLabel() const;

            private:
                std::string label;

                double confidenceLevel;

                std::vector<Data<boost::any> > objectData;
        };
    }
}

#endif // OBJECT_H
