
#ifndef INCLUDED_AABB_HEADER
#define INCLUDED_AABB_HEADER

#include "vector.hpp"

struct AABB {
    Point c;        // ���S�_
    union {         // ���a
        float r[3]; // �e���ɉ��������a
        struct {
            float rx;
            float ry;
            float rz;
        };
    };
};

#endif // !INCLUDED_AABB_HEADER
// EOF
