#ifndef NUMBER_H
#define NUMBER_H

#include <ostream>
#include <vector>

#include <boost/multiprecision/number.hpp>
#include <boost/multiprecision/cpp_int.hpp>
#include <boost/multiprecision/cpp_dec_float.hpp>

namespace Xu
{
    namespace Math
    {
        namespace Core
        {
            using Integer = boost::multiprecision::number<boost::multiprecision::cpp_int>;
            using Real = boost::multiprecision::number<boost::multiprecision::cpp_dec_float<100> >;

            using Number = Real;
        }
    }
}

#endif // NUMBER_H
