#include "Core/Fact.h"

namespace Xu
{
    namespace Core
    {
        Fact::Fact()
        {
        }

        Fact::~Fact()
        {
        }

        bool Fact::IsTrue() const
        {
            return valid;
        }

        unsigned char Fact::GetCertainty() const
        {
            return certainty;
        }
    }
}
