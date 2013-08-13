#ifndef IFUNCTION_H
#define IFUNCTION_H

namespace Xu
{
    namespace Math
    {
        namespace Calculus
        {
            class IFunction
            {
                public:
                    IFunction();
                    virtual ~IFunction();

                    virtual double Evaluate(double x) = 0;
            };
        }
    }
}

#endif // IFUNCTION_H
