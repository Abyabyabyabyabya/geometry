
#ifndef INCLUDED_RAY_HEADER_
#define INCLUDED_RAY_HEADER_

#include "vector.hpp"

struct Ray {
    Point  s; // 始点
    Vector d; // 方向(単位ベクトル)
};

#endif // !INCLUDED_RAY_HEADER_
// EOF
