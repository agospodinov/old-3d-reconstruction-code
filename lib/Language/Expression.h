#ifndef EXPRESSION_H
#define EXPRESSION_H

namespace Xu
{
    namespace Language
    {
        class Context;

        class Expression
        {
            public:
                Expression();
                virtual ~Expression();

                virtual void Evaluate(Context &context) = 0;
        };
    }
}

#endif // EXPRESSION_H
