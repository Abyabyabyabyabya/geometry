
#ifndef INCLUDED_RECTANGLE_HEADER_
#define INCLUDED_RECTANGLE_HEADER_

#include "vector.hpp"

struct Rectangle {
    Point c;     // ���S�_
    Vector u[2]; // ���[�J����
    union {      // �e���ɉ��������a
        float r[2];
        struct {
            float rx;
            float ry;
        };
    };
};

#endif // !INCLUDED_RECTANGLE_HEADER_
// EOF
