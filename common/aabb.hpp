
#ifndef INCLUDED_AABB_HEADER
#define INCLUDED_AABB_HEADER

#include "vector.hpp"

struct AABB {
    Point c;        // íÜêSì_
    union {         // îºåa
        float r[3]; // äeé≤Ç…âàÇ¡ÇΩîºåa
        struct {
            float rx;
            float ry;
            float rz;
        };
    };
};

#endif // !INCLUDED_AABB_HEADER
// EOF
