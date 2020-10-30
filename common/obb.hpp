
#ifndef INCLUDED_OBB_HEADER_
#define INCLUDED_OBB_HEADER_

#include "vector.hpp"

struct OBB {
    Point c;    // 中心点
    Point u[3]; // ローカル軸
    union {     // 各軸にそった半径
        float r[3];
        struct {
            float rx;
            float ry;
            float rz;
        };
    };
};

#if false 
struct MiniOBB {
    Point c;
    Point u[2]; // 最後の軸は cross(u[0], u[1])で必要な時に求める
    union {
        float r[3];
        struct {
            float rx;
            float ry;
            float rz;
        };
    };
};
#endif

#endif // !INCLUDED_OBB_HEADER_
// EOF
