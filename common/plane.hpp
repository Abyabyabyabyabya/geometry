
#ifndef INCLUDED_PLANE_HEADER_
#define INCLUDED_PLANE_HEADER_

#include "vector.hpp"

struct Plane {
    Point p; // 平面上の点
    Point n; // 法線
    float d; // 位相(?)
};

#endif // !INCLUDED_PLANE_HEADER_
// EOF
