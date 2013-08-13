#ifndef FACT_H
#define FACT_H

namespace Xu
{
    namespace Core
    {
        class Fact
        {
            public:
                Fact();
                virtual ~Fact();

                bool IsTrue() const;
                unsigned char GetCertainty() const;

            private:
                bool valid;
                unsigned char certainty;

        };
    }
}

#endif // FACT_H
