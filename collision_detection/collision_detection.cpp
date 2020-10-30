//
// �쐬��      : ��
// �ŏI�X�V��  : 2020/10/13
//
// �Q�l���� : �Q�[���v���O���~���O�̂��߂̃��A���^�C���Փ˔��� - Christer Ericson
//

#include <array>
#include "collition_detection.hpp"
#include <cfloat>
#include "calc.hpp"
#include "../common/matrix.hpp"

namespace {
template <class T> T clamp(const T V, const T Min, const T Max) { return V<Min ? Min : V>Max? Max : V;  }
}


// �O�p�`�̕����t�ʐς�2�{�����߂�
// �Q�l�y�[�W : 152
//
// in ABC : �O�p�`ABC
//
// ���ʂ́AABC�����v���̏ꍇ�ɐ��A�����v���̏ꍇ�ɕ�
// ABC���k�ނ��Ă���ꍇ�́A�[��
float signed2DTriArea(const Triangle& ABC) {
    return (ABC.a.x-ABC.c.x) * (ABC.b.y-ABC.c.y) - (ABC.a.y-ABC.c.y) * (ABC.b.x-ABC.c.x);
}


// AABB���m�̏Փ˔���
// �Q�l�y�[�W : 79, 80
//
// in A,B : ������s���y�A
// ret true-�Փ˂���@false-�Փ˂Ȃ�
//
// �݂��̒��_�̋����ƁA���a�̍��v���r�B
// �S�Ă̎��ɂ��āA���������a�̍��v�ȉ��̏ꍇ�ɏՓ˂��Ă���
bool testAABBAABB(const AABB& A, const AABB& B) {
    return (abs(A.c[0]-B.c[0]) <= (A.r[0]+B.r[0])) &&
           (abs(A.c[1]-B.c[1]) <= (A.r[1]+B.r[1])) &&
           (abs(A.c[2]-B.c[2]) <= (A.r[2]+B.r[2]));
}


// �����m�̏Փ˔���
// �Q�l�y�[�W : 88
//
// in A,B : ������s���y�A
// ret true-�Փ˂���@false-�Փ˂Ȃ�
//
// �݂��̋����Ɣ��a�̍��v���r�B
// ���������a�̍��v�ȉ��̏ꍇ�A�Փ˂��Ă���
bool testSphereSphere(const Sphere& A, const Sphere& B) {
    Point d = A.c - B.c;
    float dist2 = dot(d, d);
    float rad_sum = A.r + B.r;

    return dist2 <= (rad_sum * rad_sum);
}


// OBB���m�̏Փ˔���
// �Q�l�y�[�W : 102~106
//
// in A,B : ������s���y�A
// ret true-�Փ˂���@false-�Փ˂Ȃ�
//
// A��B���ꂼ��̎��ƁA���ꂼ��̎��ɐ����Ȏ��𕪗����Ƃ��ĕ�����������s���B
// �P�����A�y�ʉ��̂��߁AA�̃��[�J�����W�n�ɍ��킹�Ĕ�����s���B
// A��B���ꂼ��̎��ɂ��Ă��画����s�����ƂŁA�Փ˂��Ȃ��ꍇ�ɑ��؂�ł���\��������
bool testOBBOBB(const OBB& A, const OBB& B) {
    Matrix3x3 r, abs_r;     // B��A�̃��[�J�����W�n�ɕϊ����邽�߂̉�]�s��
    Vector t = B.c - A.c;   // A->B�̕��s�ړ��x�N�g��
    float ra, rb;           // �������ɓ��e���ꂽ�AA,B���ꂼ��̔��a

    // ���s�ړ���A�̃��[�J�����W�n�ɕϊ�
    t = Vector{dot(t, A.u[0]), dot(t, A.u[1]), dot(t, A.u[2])};

    // A.u[0], A.u[1], A.u[2] �𕪗����Ƃ�������
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            r[i][j] = dot(A.u[i], B.u[j]);
            abs_r[i][j] = abs(r[i][j]) + FLT_EPSILON; // 2�̕ӂ����s(�O�ς��[���x�N�g��)�̏ꍇ�ɔ����A�C�v�V�����̍���ǉ�
        }

        ra = A.r[i];
        rb = B.r[0]*abs_r[i][0] + B.r[1]*abs_r[i][1] + B.r[2]*abs_r[i][2];
        if(abs(t[i] > ra+rb)) return false;
    }


    // B.u[0], B.u[1], B.u[2] �𕪗����Ƃ�������
    for(int i=0; i<3; ++i) {
        ra = A.r[0]*abs_r[0][i] + A.r[1]*abs_r[1][i] + A.r[2]*abs_r[2][i];
        rb = B.r[i];
        if(abs(t[0]*r[0][i] + t[1]*r[1][i] + t[2]*r[2][i]) > ra+rb) return false;
    }


    // A.u[0] x B.u[0] �𕪗����Ƃ�������
    ra = A.r[1]*abs_r[2][0] + A.r[2]*abs_r[1][0];
    rb = B.r[1]*abs_r[0][2] + B.r[2]*abs_r[0][1];
    if(abs(t[2]*r[1][0] - t[1]*r[2][0]) > ra+rb) return false;
    // A.u[0] x B.u[1] �𕪗����Ƃ�������
    ra = A.r[1]*abs_r[2][1] + A.r[2]*abs_r[1][1];
    rb = B.r[0]*abs_r[0][2] + B.r[2]*abs_r[0][0];
    if(abs(t[2]*r[1][1] - t[1]*r[2][1]) > ra+rb) return false;
    // A.u[0] x B.u[2] �𕪗����Ƃ�������
    ra = A.r[1]*abs_r[2][2] + A.r[2]*abs_r[1][2];
    rb = B.r[0]*abs_r[0][1] + B.r[1]*abs_r[0][0];
    if(abs(t[2]*r[1][2] - t[1]*r[2][2]) > ra+rb) return false;


    // A.u[1] x B.u[0] �𕪗����Ƃ�������
    ra = A.r[0]*abs_r[2][0] + A.r[2]*abs_r[0][0];
    rb = B.r[1]*abs_r[1][2] + B.r[2]*abs_r[1][1];
    if(abs(t[0]*r[2][0] - t[2]*r[0][0]) > ra+rb) return false;
    // A.u[1] x B.u[1] �𕪗����Ƃ�������
    ra = A.r[0]*abs_r[2][1] + A.r[2]*abs_r[0][1];
    rb = B.r[0]*abs_r[1][2] + B.r[2]*abs_r[1][0];
    if(abs(t[0]*r[2][1] - t[2]*r[0][1]) > ra+rb) return false;
    // A.u[1] x B.u[2] �𕪗����Ƃ�������
    ra = A.r[0]*abs_r[2][2] + A.r[2]*abs_r[0][2];
    rb = B.r[0]*abs_r[1][1] + B.r[1]*abs_r[1][0];
    if(abs(t[0]*r[2][2] - t[2]+r[0][2]) > ra+rb) return false;


    // A.u[2] x B.u[0] �𕪗����Ƃ�������
    ra = A.r[0]*abs_r[1][0] + A.r[1]*abs_r[0][0];
    rb = B.r[1]*abs_r[2][2] + B.r[2]*abs_r[2][1];
    if(abs(t[1]*r[0][0] - t[0]*r[1][0]) > ra+rb) return false;
    // A.u[2] x B.u[1] �𕪗����Ƃ�������
    ra = A.r[0]*abs_r[1][1] + A.r[1]*abs_r[0][1];
    rb = B.r[0]*abs_r[2][2] + B.r[2]*abs_r[2][0];
    if(abs(t[1]*r[0][1] - t[0]*r[1][1]) > ra+rb) return false;
    // A.u[2] x B.u[2] �𕪗����Ƃ�������
    ra = A.r[0]*abs_r[1][2] + A.r[1]*abs_r[0][2];
    rb = B.r[0]*abs_r[2][1] + B.r[1]*abs_r[2][0];
    return abs(t[1]*r[0][2] - t[0]*r[1][2]) <= ra+rb;
}


// ���ƃJ�v�Z���̏Փ˔���
// �Q�l�y�[�W : 114, 115
//
// in A,B : ������s���y�A
// ret true-�Փ˂���@false-�Փ˂Ȃ�
//
// ���̒��S�ƃJ�v�Z���̐����̊Ԃɂ��鋗�����A�݂��̔��a�̍��v�ȉ��̏ꍇ�ɏՓ�
bool tetSphereCapsule(const Sphere& A, const Capsule& B) {
    float dist2 = sqDistPointSegment(A.c, {B.s, B.e-B.s});
    float radius = A.r + B.r;
    
    return dist2 <= radius*radius;
}


// �J�v�Z���ƃJ�v�Z���̏Փ˔���
// �Q�l�y�[�W : 114, 115
//
// in A,B : ������s���y�A
// ret true-�Փ˂���@false-�Փ˂Ȃ�
//
// �J�v�Z�������̍\���Ԃɂ����鋗�����A�݂��̔��a�̍��v�ȉ��̏ꍇ�ɏՓ�
bool testCapsuleCapsule(const Capsule& A, const Capsule& B) {
    float dist2 = sqDistSegmentSegment({A.s, A.e-A.s}, {B.s, B.e-B.s});
    float radius = A.r + B.r;

    return dist2 <= radius*radius;
}


// k-DOP���m�̏Փ˔���
// �Q�l�y�[�W : 119
//
// in AMin : k-DOP A�̍ŏ��������X�g
// in AMax : k-DOP A�̍ő勗�����X�g
// in BMin : k-DOP B�̍ŏ��������X�g
// in BMax : k-DOP B�̍ő勗�����X�g
// in N    : �����̐�(�z��̗v�f��)
// ret true-�Փ˂���@false:�Փ˂Ȃ�
bool testKDOPKDOP(const float* AMin, const float* AMax, const float* BMin, const float* BMax, const size_t N) {
    for(int i=N/2; i>0; --i) {
        if(AMin[i]>BMax[i] || AMax[i] < BMin[i])
            return false;
    }

    return true;
}


// 2D�������m�̌�������
// �Q�l�y�[�W : 153
//
// in S1 : ����1
// in S2 : ����2
// out OutPt : �����_�o�̓o�b�t�@
// out OutT  : ����1�̌����_�܂ł̊����o�̓o�b�t�@
// ret true-�Փ˂���@false:�Փ˂Ȃ�
bool test2DSegment2DSegment(const Segment2D& S1, const Segment2D& S2, Vector2D* OutPt, float* OutT) {
  // �œK��AABB����
    AABB s1_aabb;
    AABB s2_aabb;
    if(!testAABBAABB(s1_aabb, s2_aabb)) return false;

  // ��������P.153
    Vector2D a = S1.s;
    Vector2D b = S1.s + S1.d;
    Vector2D c = S2.s;
    Vector2D d = S2.s + S2.d;
    
    float a1 = signed2DTriArea(Triangle2D{a, b, d});
    float a2 = signed2DTriArea(Triangle2D{a, b, c});
    if(a1!=0.0F && a2!=0.0F && a1*a2 < 0.0F) {
        float a3 = signed2DTriArea(Triangle2D{c, d, a});
        float a4 = a3 + a2 - a1; // a1-a2 = a3-a4
        if(a3!=0.0F && a4!=0.0F && a3*a4 < 0.0F) {
            if(OutT) {
                *OutT = a3 / (a3-a4);
                if(OutPt) *OutPt = a + (b-a)*(*OutT);
            }
            else if(OutPt) {
                *OutPt = a + (b-a) * (a3/(a3-a4));
            }

            return true;
        }
    }

    return false;
}


// ���ƕ��ʂ̏Փ�(����)����
// �Q�l�y�[�W : 160, 161
//
// in S : ��
// in P : ����
// ret true-��������@false-�����Ȃ�
//
// �����ɁA���̒��S�ƕ��ʂ̊Ԃɂ��鋗�����g���Ĕ���
// ���ʂƋ����������Ă��邩�ǂ����B���S�ɂ߂肱��ł���ꍇ�́A��q�̕ʊ֐��Ŕ��肷��
// ���ʂ͐��K������Ă���K�v������( |P.n| = 1 )
bool testSpherePlane(const Sphere& S, const Plane& P) {
    float dist = dot(S.c, P.n) - P.d;
    return std::abs(dist) <= S.r;
}


// ���ƕ��ʂ̏Փ�(�߂肱��)����
// �Q�l�y�[�W : 161
//
// in S : ��
// in P : ����
// ret true-�߂荞�݂���@false-�߂荞�݂Ȃ�
//
// �������ʂ̓����ɂ��邩�ǂ����B�������Ă���ꍇ�͂߂荞�݂Ȃ��Ɣ���B
// ���ʂ͐��K������Ă���K�v������
bool testInsideSpherePlane(const Sphere& S, const Plane& P) {
    float dist = dot(S.c, P.n) - P.d;
    return dist < -S.r;
}


// ���ƕ��ʂ̏Փ�(�����A�������͂߂荞��)����
// �Q�l�y�[�W : 161
//
// in S : ��
// in P : ����
// ret true-�Փ˂���@false-�Փ˂Ȃ�
//
// ���ʂ͐��K������Ă���K�v������
bool testSphereHarfSpace(const Sphere& S, const Plane& P) {
    float dist = dot(S.c, P.n) - P.d;
    return dist <= S.r;
}


// OBB�ƕ��ʂ̏Փ�(����)����
// �Q�l�y�[�W : 161, 162, 163
//
// in B : OBB
// in P : ����
// ret true-��������@false-�����Ȃ�
//
// ����������ɂ�����
// �������͕��ʂ̖@��(P.n)�ɕ��s�Ȑ���1�̂�
// ���ʂ͐��K������Ă��Ȃ��Ƃ��悢(�ŏI�I�Ȕ�r�i�K�ŉe�����Ȃ�)
bool testOBBPlane(const OBB& B, const Plane& P) {
    float r = B.r[0] * std::abs(dot(P.n, B.u[0])) +
        B.r[1] + std::abs(dot(P.n, B.u[1])) +
        B.r[2] + std::abs(dot(P.n, B.u[2]));
    float s = dot(P.n, B.c) - P.d;
    return std::abs(s) <= r;
}


// OBB�ƕ��ʂ̏Փ�(�߂荞��)����
// �Q�l�y�[�W : 162, 163
//
// in B : OBB
// in P : ����
// ret true-�߂荞�݂���@false-�߂荞�݂Ȃ�
bool testInsideOBBPlane(const OBB& B, const Plane& P) {
    float r = B.r[0] * std::abs(dot(P.n, B.u[0])) +
        B.r[1] + std::abs(dot(P.n, B.u[1])) +
        B.r[2] + std::abs(dot(P.n, B.u[2]));
    float s = dot(P.n, B.c) - P.d;
    return s < -r;
}


// OBB�ƕ��ʂ̏Փ�(�����A�߂荞��)����
// �Q�l�y�[�W : 162, 163
//
// in B : OBB
// in P : ����
// ret true-�Փ˂���@false-�Փ˂Ȃ�
bool testOBBHarfSpace(const OBB& B, const Plane& P) {
    float r = B.r[0] * std::abs(dot(P.n, B.u[0])) +
        B.r[1] + std::abs(dot(P.n, B.u[1])) +
        B.r[2] + std::abs(dot(P.n, B.u[2]));
    float s = dot(P.n, B.c) - P.d;
    return s <= r;
}


// AABB�ƕ��ʂ̏Փ�(����)����
// �Q�l�y�[�W : 163
//
// in B : AABB
// in P : ����
// ret true-��������@false-�����Ȃ�
bool testAABBPlane(const AABB& B, const Plane& P) {
    float r = B.r[0] * std::abs(P.n[0]) +
        B.r[1] * std::abs(P.n[1]) + 
        B.r[2] * std::abs(P.n[2]);
    float s = dot(P.n, B.c) - P.d;
    return std::abs(s) <= r;
}


// AABB�ƕ��ʂ̏Փ�(�߂荞��)����
//
// in B : AABB
// in P : ����
// ret true-�߂荞�݂���@false-�߂荞�݂Ȃ�
bool testInsideAABBPlane(const AABB& B, const Plane& P) {
    float r = B.r[0] * std::abs(P.n[0]) +
        B.r[1] * std::abs(P.n[1]) + 
        B.r[2] * std::abs(P.n[2]);
    float s = dot(P.n, B.c) - P.d;
    return s < -r;
}


// AABB�ƕ��ʂ̏Փ�(�����A�߂荞��)����
//
// in B : AABB
// in P : ����
// ret true-�Փ˂���@false-�Փ˂Ȃ�
bool testAABBHarfSpace(const AABB& B, const Plane& P) {
    float r = B.r[0] * std::abs(P.n[0]) +
        B.r[1] * std::abs(P.n[1]) + 
        B.r[2] * std::abs(P.n[2]);
    float s = dot(P.n, B.c) - P.d;
    return s <= r;
}


// ����AABB�̏Փ˔���
// �Q�l�y�[�W : 165, 166
//
// in S : ��
// in B : AABB
// ret true-�Փ˂���@false-�Փ˂Ȃ�
bool testSphereAABB(const Sphere& S, const AABB& B) {
    float sq_dist = sqDistPointAABB(S.c, B);
    return sq_dist <= S.r*S.r;
}


// ����OBB�̏Փ˔���
// �Q�l�y�[�W : 166, 167
//
// in S : ��
// in B : OBB
// ret true-�Փ˂���@false-�Փ˂Ȃ�
bool testSphereOBB(const Sphere& S, const OBB& B) {
    float sq_dist = sqDistPointOBB(S.c, B);
    return sq_dist <= S.r*S.r;
}


// ���ƎO�p�`�̏Փ˔���
// �Q�l�y�[�W : 167
//
// in S : ��
// in T : �O�p�`
// ret true-�Փ˂���@false-�Փ˂Ȃ�
bool testSphereTriangle(const Sphere& S, const Triangle& T) {
    Point c = closestPtPointTriangle(S.c, T);
    return dot(c, c) <= S.r*S.r;
}


// �O�p�`��AABB�̏Փ˔���
// �Q�l�y�[�W : 169~172
//
// in T : �O�p��
// in B : AABB
// ret true-�Փ˂���@false-�Փ˂Ȃ�
//
// ����������ɂ�����
// 1. AABB�̖ʖ@��(3��)
//      AABB�ƁA�O�p�`��AABB�ɂ�锻��Ɠ���
// 2. �O�p�`�̖ʖ@��(1��)
//      AABB�ƎO�p�`�̕��ʂɂ���������Ɠ���
// 3. AABB�̎��ƎO�p�`�̕� �̊O��(9�� ��3*3)
//      ��������ƕ�����������s��
//
// �œK�� : 3->1->2 �̏��Ԃŏ������s��
bool testTriangleAABB(Triangle T, const AABB& B) {
    // AABB�̒��S�����_�Ƃ��邱�ƂŌv�Z��P����
    T.a = T.a - B.c;
    T.b = T.b - B.c;
    T.c = T.c - B.c;

    Vector f[3] // �O�p�`�̕�
    { T.b - T.a,
      T.c - T.b,
      T.a - T.c };


    // 3 - ����������
    float p0, p1, p2, r;
    Vector u[3] // AABB�̖ʖ@��
    { {1.0F, 0.0F, 0.0F},
      {0.0F, 1.0F, 0.0F},
      {0.0F, 0.0F, 1.0F} };


    // 1 - AABB vs AABB
    auto max = [](const float V1, const float V2, const float V3) {
        return V1 > V2 ?
                   V1 > V3 ? V1 : V3
               :   V2 > V3 ? V2 : V3;
    };
    auto min = [](const float V1, const float V2, const float V3) {
        return V1 < V2 ?
                   V1 < V3 ? V1 : V3
               :   V2 < V3 ? V2 : V3;
    };
    if(max(T.a.x, T.b.x, T.c.x)<-B.rx || min(T.a.x, T.b.x, T.c.x)>B.rx) return false;
    if(max(T.a.y, T.b.y, T.c.y)<-B.ry || min(T.a.y, T.b.y, T.c.y)>B.ry) return false;
    if(max(T.a.z, T.b.z, T.c.z)<-B.rz || min(T.a.z, T.b.z, T.c.z)>B.rz) return false;


    // 2 - AABB vs Plane
    Plane p;
    // p.p = Point{}; testAABBPlane(...)�ł͎g�p���Ȃ��̂ŏ������s�v
    p.n = cross(f[0], f[1]);
    p.d = dot(p.n, T.a);
    return testAABBPlane(B, p);;
}


// ���ʏ�ɂ���A�_�ɍł��߂��_�����߂�
// �Q�l�y�[�W : 127
//
// in Q : �_
// in P : ����
//
// ���ʂ̖@���͐��K������Ă���K�v������
Point closestPtPointPlane(const Point& Q, const Plane& P) {
    float t = dot(P.n, Q) - P.d;
    return Q - P.n*t;
}


// �^����ꂽ�_�ƕ��ʂ̊Ԃɂ��鋗�������߂�
// �Q�l�y�[�W : 127
//
// in Q : �_
// in P : ����
//
// ���ʂ̖@���͐��K������Ă���K�v������
float distPointPlane(const Point& Q, const Plane& P) {
    return dot(Q, P.n) - P.d;
}


// �_�Ɛ����̍Őڋߓ_�����߂�
// �Őڋߓ_�͐�����ɂ���
// �Q�l�y�[�W : 128
//
// in P : �_
// in S : ����
// ret �Őڋߓ_
Point closestPtPointSegment(const Point& P, const Segment& S) {
    return S.s + S.d*percentageSegmentToClosestPt(P, S);
}


// �_�Ɛ����̍Őڋߓ_�܂ł́A�����̋����̊��������߂�
// ������[0.0F : 1.0F]�͈̔͂ɃN�����v�����
// �Q�l�y�[�W : 128
//
// in P : �_
// in S : ����
// ret �Őڋߓ_�܂ł́A�����̋����̊���
//
// �Őڋߓ_d(t) = S.s + (S.d * t) <- t�����߂�
float percentageToClosestPtPointSegment(const Point& P, const Segment& S) {
    float t = dot(P-S.s, S.d) / dot(S.d, S.d);
    return clamp(t, 0.0F, 1.0F);
}


// �_�Ɛ����̊Ԃɂ��鋗���̕��������߂�
// �����́A�݂��̊Ԃɂ���ł��������������w��
// �Q�l�y�[�W : 130
//
// in P : �_
// in S : ����
// ret �����̕���
float sqDistPointSegment(const Point& P, const Segment& S) {
    Vector ac = P - S.s;
    float e = dot(ac, S.d);

    // S.s(�����̎n�_)���Őڋߓ_�̏ꍇ
    if(e <= 0.0F) return dot(ac, ac);

    float f = dot(S.d, S.d);

    // S.s+S.d(�����̏I�_)���Őڋߓ_�̏ꍇ
    if(e >= f) {
        Vector bc = P - (S.s+S.d);
        return dot(bc, bc);
    }

    // �Őڋߓ_��S��ɂ���ꍇ
    return dot(ac, ac) - e*e / f;
}


// �_��AABB�̍Őڋߓ_�����߂�
// �Őڋߓ_��AABB�̏�ɂ���
// �Q�l�y�[�W : 130, 131
//
// in P : �_
// in B : AABB
// ret �Őڋߓ_
//
// x,y,z �e���ɂ��āA�_��AABB�͈͓̔��ɃN�����v����
Point closestPtPointAABB(Point P, const AABB& B) {
    for(int i=0; i<3; ++i) {
        float temp = B.c[i]-B.r[i];
        if(P[i] < temp) P[i] = temp;  else {

        temp = B.c[i]+B.r[i];
        if(P[i] > temp) P[i] = temp;  }
    }

    return P;
}


// �_��AABB�̊Ԃɂ��鋗���̕��������߂�
// �����́A�݂��̊Ԃɂ���ł��������������w��
// �Q�l�y�[�W : 131, 132
// 
// in P : �_
// in B : AABB
// ret �����̕���
//
// �e���ɂ��āA�]���ȋ�����2������߂ĉ��Z���Ă���
// �ŏI�I�ɎO�����̒藝�Ad^2 = x^2 + y^2 + z^2 �����߂���
float sqDistPointAABB(const Point& P, const AABB& B) {
    float sq_dist{};
    for(int i=0; i<3; ++i) {
        float temp = B.c[i] - B.r[i];
        if(P[i] < temp) sq_dist += (temp-P[i]) * (temp-P[i]);  else {
        
        temp = B.c[i] + B.r[i];
        if(P[i] > temp) sq_dist += (P[i]-temp) * (P[i]-temp);  }
    }

    return sq_dist;
}


// �_��OBB�̍Őڋߓ_�����߂�
// �Q�l�y�[�W : 133
//
// in P : �_
// in B : OBB
// ret �Őڋߓ_
//
// �_��OBB�̊e���ɉ����āA�]���ȋ��������ړ�
// �_��AABB�̍Őڋߓ_�����߂�̂ƍl�����͓���
Point closestPtPointOBB(Point P, const OBB& B) {
    Vector d = P - B.c;
    P = B.c;
    for(int i=0; i<3; ++i) {
        float dist = dot(d, B.u[i]);
        if(dist > B.r[i])       dist = B.r[i];
        else if(dist < -B.r[i]) dist = -B.r[i];

        P = P + B.u[i] * dist;
    }

    return P;
}


// �_��OBB�̊Ԃɂ��鋗���̕��������߂�
// �����́A�݂��̊Ԃɂ���ł��������������w��
// �Q�l�y�[�W : 134
//
// in P : �_
// in B : OBB
// ret �����̕���
float sqDistPointOBB(const Point& P, const OBB& B) {
    float sq_dist{};
    Vector v = P - B.c;
    for(int i=0; i<3; ++i) {
        float d = dot(v, B.u[i]);
        float excess{};
        if(d > B.r[i])        excess = d - B.r[i];
        else if( d < -B.r[i]) excess = d + B.r[i];

        sq_dist += excess*excess;
    }

    return sq_dist;
}


// �_�ƒ����`�̍Őڋߓ_�����߂�
// �Q�l�y�[�W : 135
//
// in P : �_
// in R : �����`
// ret �Őڋߓ_
//
// closestPtPointOBB�̉��p
Point closestPtPointRect(Point P, const Rectangle& R) {
    Vector d = P - R.c;
    P = R.c;
    for(int i=0; i<2; ++i) {
        float dist = dot(d, R.u[i]);
        if(dist > R.r[i])       dist = R.r[i];
        else if(dist < -R.r[i]) dist = -R.r[i];

        P = P + R.u[i]*dist;
    }

    return P;
}


// �_�ƒ����`�̊Ԃɂ��鋗���̕��������߂�
// �Q�l�y�[�W : �Ȃ�
//
// in P : �_
// in R : �����`
// ret �����̕���
float sqDistPointRectangle(const Point& P, const Rectangle& R) {
    OBB b {
        R.c,
        {R.u[0], R.u[1], cross(R.u[0], R.u[1])},
        {R.r[0], R.r[1], 0.0F}
    };
    return sqDistPointOBB(P, b);
}


// �_�ƎO�p�`�̍Őڋߓ_�����߂�
// �Q�l�y�[�W : 141, 142
//
// in P : �_
// in T : �O�p�`
// ret �Őڋߓ_
//
// �Őڋߓ_�͎O�p�`�ɑ�����
// �_���ǂ̃{���m�C�̈�ɑ����邩���l����
// case 1: ���_�̈�̏ꍇ�A�Ή����钸�_���Őڋߓ_
// case 2: �ӗ̈�̏ꍇ�A�Ή�����ӂɓ_�𐂒��ˉe�����_���Őڋߓ_
// case 3: ���_�A�ӂ̂ǂ̃{���m�C�̈�ɂ������Ȃ��ꍇ�A�_���O�p�`�ɐ����ˉe�����_���Őڋߓ_
Point closestPtPointTriangle(const Point& P, const Triangle& T) {
    // P��T.a�̒��_�̈�ɂ��邩
    Vector ab = T.b - T.a;
    Vector ac = T.c - T.a;
    Vector ap = P - T.a;
    float d1 = dot(ab, ap);
    float d2 = dot(ac, ap);
    if(d1 <= 0.0F && d2 <= 0.0F) return T.a;    // case 1

    // P��T.b�̒��_�̈�ɂ��邩
    Vector bp = P - T.b;
    float d3 = dot(ab, bp);
    float d4 = dot(ac, bp);
    if(d3 >= 0.0F && d4 <= d3) return T.b;      // case 1

    // P������T.a->T.b�̕ӗ̈�ɂ��邩
    float vc = d1*d4 - d3*d2;
    if(vc <= 0.0F && d1 >= 0.0F && d3 <= 0.0F)
        return T.a + ab * /*v=*/(d1/(d1-d3));   // case 2

    // P��C�̒��_�̈�ɂ��邩
    Vector cp = P - T.c;
    float d5 = dot(ab, cp);
    float d6 = dot(ac, cp);
    if(d6 >= 0.0F && d5 <= d6) return T.c;      // case 1

    // P������T.a->T.c�̕ӗ̈�ɂ��邩
    float vb = d5*d2 - d1*d6;
    if(vb <= 0.0F && d2 >= 0.0F && d6 <= 0.0F)
        return T.a + ac * /*v=*/(d2/d2-d6);     // case 2

    // P������T.b->T.c�̕ӗ̈�ɂ��邩
    float va = d3*d6 - d5*d4;
    if(va <= 0.0F && (d4-d3) >= 0.0F && (d5-d6) >= 0.0F)
        return T.b + (T.c-T.b) * /*v=*/((d4-d3)/((d4-d3) + (d5-d6))); // case 2

    float denom = 1.0F / (va+vb+vc);
    float v = vb * denom;
    float w = vc * denom;
    return T.a + ab*v + ac*w;                   // case 3
}


// �����Ɛ����̍Őڋߓ_�܂ł́A���ꂼ��̋����̊��������߂�
//
// in S1 : ����1
// in S2 : ����2
// out C1 : ����1�̍Őڋߓ_�܂ł̊���
// out C2 : ����2�̍Őڋߓ_�܂ł̊���
void closestPtSegmentSegment(const Segment& S1, const Segment& S2, float* const C1, float* const C2) {
    Vector r = S1.s - S2.s;
    float a = dot(S1.d, S1.d);
    float e = dot(S2.d, S2.d);
    float f = dot(S2.d, r);

    if(a <= FLT_EPSILON && e <= FLT_EPSILON) {
      // �����̐������_�ɏk��
        *C1 = *C2 = 0.0F;
    }
    else if(a <= FLT_EPSILON) {
      // ����S1���_�ɏk��
        *C1 = 0.0F;
        *C2 = clamp(f/e, 0.0F, 1.0F);
    }
    else if(e <= FLT_EPSILON) {
      // ����S2���_�ɏk��
        *C2 = 0.0F;
        *C1 = clamp((-dot(S1.d, r))/a, 0.0F, 1.0F);
    }
    else {
      // �����Ɛ����̍Őڋߓ_�܂ł̊��������߂�
        float b = dot(S1.d, S2.d);
        float c = dot(S1.d, r);
        float denom = a*e - b*b;

        if(denom != 0.0F) {
            *C1 = clamp((b*f - c*e)/denom, 0.0F, 1.0F);
        }
        else {
            *C1 = 0.0F;
        }

        *C2 = (b*(*C1) + f) / e;

        if(*C2 < 0.0F) {
            *C2 = 0.0F;
            *C1 = clamp(-c/a, 0.0F, 1.0F);
        }
        else if(*C2 > 1.0F) {
            *C2 = 1.0F;
            *C1 = clamp((b-c)/a, 0.0F, 1.0F);
        }
    }
}
// �����Ɛ����̍Őڋߓ_�����߂�
//
// in S1 : ����1
// in S2 : ����2
// out C1 : ����1��̍Őڋߓ_
// out C2 : ����2��̍Őڋߓ_
void closestPtSegmentSegment(const Segment& S1, const Segment& S2, Point* const C1, Point* const C2) {
    float s, t;
    closestPtSegmentSegment(S1, S2, &s, &t);
    *C1 = S1.s + S1.d*s;
    *C2 = S2.s + S2.d*t;
}


// �����Ɛ����̊Ԃɂ��鋗���̕��������߂�
//
// in S1 : ����1
// in S2 : ����2
// ret �����̕���
float sqDistSegmentSegment(const Segment& S1, const Segment& S2) {
    Point c1;
    Point c2;
    closestPtSegmentSegment(S1, S2, &c1, &c2);
    
    c1 = c1 - c2;
    return dot(c1, c1);
}


// �����ƎO�p�`�̍Őڋߓ_�����߂�
// �Q�l�y�[�W : 154, 155
//
// in S : ����
// in T : �O�p�`
// ret �Őڋߓ_
//
// �Őڋߓ_�͎O�p�`�ɑ�����
// �������̍l������g�ݍ��킹�̒��ŁA�ł��߂������̓_���Őڋߓ_
// 0. �����ƎO�p�`�̕�ab
// 1. �����ƎO�p�`�̕�bc
// 2. �����ƎO�p�`�̕�ca
// 3. �����̎n�_�ƎO�p�`�̕���
// 4. �����̏I�_�ƎO�p�`�̕���
//
// ����Ȃ�œK���̗]�n����( �ڍׂ͎Q�l�y�[�W���Q�� ) 
Point closestPtSegmentTriangle(const Segment& S, const Triangle& T) {
    Point points[5];
    float sq_dists[5];

    sq_dists[0] = sqDistSegmentSegment(S, {T.b-T.a}); // 0
    sq_dists[1] = sqDistSegmentSegment(S, {T.c-T.b}); // 1
    sq_dists[2] = sqDistSegmentSegment(S, {T.a-T.c}); // 2

    points[3] = closestPtPointTriangle(S.s, T);       // 3
    Vector temp = points[3] - S.s;
    sq_dists[3] = dot(temp, temp);

    points[4] = closestPtPointTriangle(S.s+S.d, T);   // 4
    temp = points[4] - (S.s+S.d);
    sq_dists[4] = dot(temp, temp);

    // �ŏ��̋����ƂȂ�g�ݍ��킹�����߂�
    size_t min_idx=0;
    for(int i=1; i<5; ++i) {
        if(sq_dists[i] < sq_dists[min_idx])
            min_idx = i;
    }

    // �ŏI�I�ɋ߂��_�̂݋��߂�( 3,4�̑g�ݍ��킹�́A��ŋ��߂��Ă��� )
    switch(min_idx) {
    case 0 : closestPtSegmentSegment(S, Segment{T.b-T.a}, /*dummy*/&temp, &points[0]); break;
    case 1 : closestPtSegmentSegment(S, Segment{T.c-T.b}, /*dummy*/&temp, &points[1]); break;
    case 2 : closestPtSegmentSegment(S, Segment{T.a-T.c}, /*dummy*/&temp, &points[2]); break;
    };

    return points[min_idx];
}

// �O�p�`�ƎO�p�`�̍Őڋߓ_�����߂�
// �Q�l�y�[�W : 156, 157
//
// in A : �O�p�`1
// in B : �O�p�`2
// out OutP1 : �O�p�`1��ɂ���Őڋߓ_�̏o�̓o�b�t�@
// out OutP2 : �O�p�`2��ɂ���Őڋߓ_�̏o�̓o�b�t�@
// ret �Őڋߓ_�y�A�̊Ԃɂ��鋗���̕���
//
// �������̍l������g�ݍ��킹�̒��ŁA�ł��߂������̓_���Őڋߓ_
// 0. �e�O�p�`�̂��ׂĂ̕ӓ��m(9�p�^�[��)
// 1. ����̒��_�Ƒ����̎O�p�`(6�p�^�[��)
//
// ����Ȃ�挒���̌��オ���߂���
// ...�O�p�`���������Ă���ꍇ�A���s�ɏd�Ȃ��Ă���ꍇ
float closestPtTriangleTriangle(const Triangle& A, const Triangle& B, Point* OutP1, Point* OutP2) {
    struct ClosestPair{
        Point p[2];
        float sq_dist;
    };
    std::array<ClosestPair, 15U> closest_points;    // �Őڋߓ_���̃y�A���i�[���� (15 = 9+6 ���ڂ����͏�L�ڍׂ��Q��)

    auto segmentInTri = [](const Triangle& T, const size_t SegIdx /*1~3*/)->Segment {
        const size_t kStartIdx = SegIdx-1; // �n�_
        const size_t kEndIdx   = SegIdx%3; // �I�_

        // SegIdx�̒l�ɉ����� 0->1, 1->2, 2->0 �̑g�ݍ��킹�łł��������Ԃ�
        return Segment{T.p[kStartIdx], T.p[kEndIdx]-T.p[kStartIdx]};
    };
    // 0. �e�O�p�`�̐������m
    for(int i=0; i<3; ++i) {
        const Segment kT1Seg = segmentInTri(A, i);
        for(int j=0; j<3; ++j) {
            const Segment kT2Seg = segmentInTri(B, j);
            const size_t kOutIdx = (i*3)+j;

            closestPtSegmentSegment(kT1Seg, kT2Seg, &closest_points[kOutIdx].p[0], &closest_points[kOutIdx].p[1]);
            Point temp = closest_points[kOutIdx].p[1]-closest_points[kOutIdx].p[0];
            closest_points[kOutIdx].sq_dist = dot(temp, temp);
        }
    }

    // 1. ���_�ƎO�p�`
    const Triangle* tris[] {&A, &B};
    for(int i=0; i<2; ++i) {
        for(int j=0; j<3; ++j) {
            const size_t kOutIdx = 9+(i*3)*j;

            closest_points[kOutIdx].p[i] = closestPtPointTriangle(tris[!i]->p[j], *tris[i]);
            closest_points[kOutIdx].p[!i] = tris[!i]->p[j];
            Point temp = closest_points[kOutIdx].p[!i]-closest_points[kOutIdx].p[i];
            closest_points[kOutIdx].sq_dist = dot(temp, temp);
        }
    }


    // �ł��������������_�̃y�A��������
    size_t min_idx = 0;
    for(int i=1; i<closest_points.size(); ++i) {
        if(closest_points[i].sq_dist < closest_points[min_idx].sq_dist)
            min_idx = i;
    }
    
    *OutP1 = closest_points[min_idx].p[0];
    *OutP2 = closest_points[min_idx].p[1];
    return closest_points[min_idx].sq_dist;
}
// EOF
