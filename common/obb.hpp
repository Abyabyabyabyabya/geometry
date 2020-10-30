
#ifndef INCLUDED_OBB_HEADER_
#define INCLUDED_OBB_HEADER_

#include "vector.hpp"

struct OBB {
    Point c;    // ���S�_
    Point u[3]; // ���[�J����
    union {     // �e���ɂ��������a
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
    Point u[2]; // �Ō�̎��� cross(u[0], u[1])�ŕK�v�Ȏ��ɋ��߂�
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
