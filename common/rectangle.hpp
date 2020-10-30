
#ifndef INCLUDED_RECTANGLE_HEADER_
#define INCLUDED_RECTANGLE_HEADER_

#include "vector.hpp"

struct Rectangle {
    Point c;     // 中心点
    Vector u[2]; // ローカル軸
    union {      // 各軸に沿った半径
        float r[2];
        struct {
            float rx;
            float ry;
        };
    };
};

#endif // !INCLUDED_RECTANGLE_HEADER_
// EOF
