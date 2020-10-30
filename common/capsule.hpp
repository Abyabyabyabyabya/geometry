
#ifndef INCLUDED_CAPSULE_HEADER_
#define INCLUDED_CAPSULE_HEADER_

#include "vector.hpp"

struct Capsule {
    Point s; // 線分の開点
    Point e; // 線分の終点
    float r; // 半径
};

#endif // !INCLUDED_CAPSULE_HEADER_
// EOF
