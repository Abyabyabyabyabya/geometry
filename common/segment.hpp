
#ifndef INCLUDED_SEGMENT_HEADER_
#define INCLUDED_SEGMENT_HEADER_

#include "vector.hpp"

struct Segment {
    Point s;  // 始点
    Vector d; // 距離
};

struct Segment2D {
    Vector2D s; // 始点
    Vector2D d; // 距離
};

#endif // !INCLUDED_SEGMENT_HEADER_
// EOF
