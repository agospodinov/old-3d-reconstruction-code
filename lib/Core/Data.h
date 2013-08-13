#ifndef OBJECTDATA_H
#define OBJECTDATA_H

#include <memory>

namespace Xu
{
    namespace Core
    {
        template <typename T>
        class Data
        {
            public:
                Data(const T &dataItem)
                    : dataItem(dataItem)
                {
                }

                ~Data()
                {
                }

                inline const T &GetItem() const
                {
                    return dataItem;
                }

                inline T &GetItem()
                {
                    return dataItem;
                }

            private:
                T dataItem;
        };
    }
}

#endif // OBJECTDATA_H
