
#ifndef INCLUDED_VECTOR_HEADER_
#define INCLUDED_VECTOR_HEADER_

#include <cstdint>

struct Vector2D {
    union {
        float p[2];
        struct {
            float x;
            float y;
        };
    };

    float& operator[](const size_t N) noexcept { return p[N]; }
    float operator[](const size_t N) const noexcept { return p[N]; }
};

struct Vector3D {
    union {
        float p[3];
        struct {
            float x;
            float y;
            float z;
        };
    };

    float& operator[](const size_t N) noexcept { return p[N]; }
    float operator[](const size_t N) const noexcept { return p[N]; }
};

using Vector = Vector3D;
using Point  = Vector;

#endif // !INCLUDED_VECTOR_HEADER_
// EOF
