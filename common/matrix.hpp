
#ifndef INCLUDED_MATRIX_HEADER_
#define INCLUDED_MATRIX_HEADER_

#include <cstdint>
#include "vector.hpp"

struct Matrix3x3 {
    union {
        float m[3][3];
        Vector v[3];
    };

    Vector& operator[](const size_t N) { return v[N]; }
    const Vector& operator[](const size_t N) const { return v[N]; }
};

#endif // !INCLUDED_MATRIX_HEADER_
// EOF
