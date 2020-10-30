//
// 作成者      : 板場
// 最終更新日  : 2020/10/13
//
// 参考文献 : ゲームプログラミングのためのリアルタイム衝突判定 - Christer Ericson
//

#include <array>
#include "collition_detection.hpp"
#include <cfloat>
#include "calc.hpp"
#include "../common/matrix.hpp"

namespace {
template <class T> T clamp(const T V, const T Min, const T Max) { return V<Min ? Min : V>Max? Max : V;  }
}


// 三角形の符号付面積の2倍を求める
// 参考ページ : 152
//
// in ABC : 三角形ABC
//
// 結果は、ABCが時計回りの場合に正、反時計回りの場合に負
// ABCが縮退している場合は、ゼロ
float signed2DTriArea(const Triangle& ABC) {
    return (ABC.a.x-ABC.c.x) * (ABC.b.y-ABC.c.y) - (ABC.a.y-ABC.c.y) * (ABC.b.x-ABC.c.x);
}


// AABB同士の衝突判定
// 参考ページ : 79, 80
//
// in A,B : 判定を行うペア
// ret true-衝突あり　false-衝突なし
//
// 互いの中点の距離と、半径の合計を比較。
// 全ての軸について、距離が半径の合計以下の場合に衝突している
bool testAABBAABB(const AABB& A, const AABB& B) {
    return (abs(A.c[0]-B.c[0]) <= (A.r[0]+B.r[0])) &&
           (abs(A.c[1]-B.c[1]) <= (A.r[1]+B.r[1])) &&
           (abs(A.c[2]-B.c[2]) <= (A.r[2]+B.r[2]));
}


// 球同士の衝突判定
// 参考ページ : 88
//
// in A,B : 判定を行うペア
// ret true-衝突あり　false-衝突なし
//
// 互いの距離と半径の合計を比較。
// 距離が半径の合計以下の場合、衝突している
bool testSphereSphere(const Sphere& A, const Sphere& B) {
    Point d = A.c - B.c;
    float dist2 = dot(d, d);
    float rad_sum = A.r + B.r;

    return dist2 <= (rad_sum * rad_sum);
}


// OBB同士の衝突判定
// 参考ページ : 102~106
//
// in A,B : 判定を行うペア
// ret true-衝突あり　false-衝突なし
//
// AとBそれぞれの軸と、それぞれの軸に垂直な軸を分離軸として分離軸判定を行う。
// 単純化、軽量化のため、Aのローカル座標系に合わせて判定を行う。
// AとBそれぞれの軸についてから判定を行うことで、衝突がない場合に足切りできる可能性が高い
bool testOBBOBB(const OBB& A, const OBB& B) {
    Matrix3x3 r, abs_r;     // BをAのローカル座標系に変換するための回転行列
    Vector t = B.c - A.c;   // A->Bの平行移動ベクトル
    float ra, rb;           // 分離軸に投影された、A,Bそれぞれの半径

    // 平行移動をAのローカル座標系に変換
    t = Vector{dot(t, A.u[0]), dot(t, A.u[1]), dot(t, A.u[2])};

    // A.u[0], A.u[1], A.u[2] を分離軸とした判定
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            r[i][j] = dot(A.u[i], B.u[j]);
            abs_r[i][j] = abs(r[i][j]) + FLT_EPSILON; // 2つの辺が平行(外積がゼロベクトル)の場合に備え、イプシロンの項を追加
        }

        ra = A.r[i];
        rb = B.r[0]*abs_r[i][0] + B.r[1]*abs_r[i][1] + B.r[2]*abs_r[i][2];
        if(abs(t[i] > ra+rb)) return false;
    }


    // B.u[0], B.u[1], B.u[2] を分離軸とした判定
    for(int i=0; i<3; ++i) {
        ra = A.r[0]*abs_r[0][i] + A.r[1]*abs_r[1][i] + A.r[2]*abs_r[2][i];
        rb = B.r[i];
        if(abs(t[0]*r[0][i] + t[1]*r[1][i] + t[2]*r[2][i]) > ra+rb) return false;
    }


    // A.u[0] x B.u[0] を分離軸とした判定
    ra = A.r[1]*abs_r[2][0] + A.r[2]*abs_r[1][0];
    rb = B.r[1]*abs_r[0][2] + B.r[2]*abs_r[0][1];
    if(abs(t[2]*r[1][0] - t[1]*r[2][0]) > ra+rb) return false;
    // A.u[0] x B.u[1] を分離軸とした判定
    ra = A.r[1]*abs_r[2][1] + A.r[2]*abs_r[1][1];
    rb = B.r[0]*abs_r[0][2] + B.r[2]*abs_r[0][0];
    if(abs(t[2]*r[1][1] - t[1]*r[2][1]) > ra+rb) return false;
    // A.u[0] x B.u[2] を分離軸とした判定
    ra = A.r[1]*abs_r[2][2] + A.r[2]*abs_r[1][2];
    rb = B.r[0]*abs_r[0][1] + B.r[1]*abs_r[0][0];
    if(abs(t[2]*r[1][2] - t[1]*r[2][2]) > ra+rb) return false;


    // A.u[1] x B.u[0] を分離軸とした判定
    ra = A.r[0]*abs_r[2][0] + A.r[2]*abs_r[0][0];
    rb = B.r[1]*abs_r[1][2] + B.r[2]*abs_r[1][1];
    if(abs(t[0]*r[2][0] - t[2]*r[0][0]) > ra+rb) return false;
    // A.u[1] x B.u[1] を分離軸とした判定
    ra = A.r[0]*abs_r[2][1] + A.r[2]*abs_r[0][1];
    rb = B.r[0]*abs_r[1][2] + B.r[2]*abs_r[1][0];
    if(abs(t[0]*r[2][1] - t[2]*r[0][1]) > ra+rb) return false;
    // A.u[1] x B.u[2] を分離軸とした判定
    ra = A.r[0]*abs_r[2][2] + A.r[2]*abs_r[0][2];
    rb = B.r[0]*abs_r[1][1] + B.r[1]*abs_r[1][0];
    if(abs(t[0]*r[2][2] - t[2]+r[0][2]) > ra+rb) return false;


    // A.u[2] x B.u[0] を分離軸とした判定
    ra = A.r[0]*abs_r[1][0] + A.r[1]*abs_r[0][0];
    rb = B.r[1]*abs_r[2][2] + B.r[2]*abs_r[2][1];
    if(abs(t[1]*r[0][0] - t[0]*r[1][0]) > ra+rb) return false;
    // A.u[2] x B.u[1] を分離軸とした判定
    ra = A.r[0]*abs_r[1][1] + A.r[1]*abs_r[0][1];
    rb = B.r[0]*abs_r[2][2] + B.r[2]*abs_r[2][0];
    if(abs(t[1]*r[0][1] - t[0]*r[1][1]) > ra+rb) return false;
    // A.u[2] x B.u[2] を分離軸とした判定
    ra = A.r[0]*abs_r[1][2] + A.r[1]*abs_r[0][2];
    rb = B.r[0]*abs_r[2][1] + B.r[1]*abs_r[2][0];
    return abs(t[1]*r[0][2] - t[0]*r[1][2]) <= ra+rb;
}


// 球とカプセルの衝突判定
// 参考ページ : 114, 115
//
// in A,B : 判定を行うペア
// ret true-衝突あり　false-衝突なし
//
// 球の中心とカプセルの線分の間にある距離が、互いの半径の合計以下の場合に衝突
bool tetSphereCapsule(const Sphere& A, const Capsule& B) {
    float dist2 = sqDistPointSegment(A.c, {B.s, B.e-B.s});
    float radius = A.r + B.r;
    
    return dist2 <= radius*radius;
}


// カプセルとカプセルの衝突判定
// 参考ページ : 114, 115
//
// in A,B : 判定を行うペア
// ret true-衝突あり　false-衝突なし
//
// カプセル内部の構造間における距離が、互いの半径の合計以下の場合に衝突
bool testCapsuleCapsule(const Capsule& A, const Capsule& B) {
    float dist2 = sqDistSegmentSegment({A.s, A.e-A.s}, {B.s, B.e-B.s});
    float radius = A.r + B.r;

    return dist2 <= radius*radius;
}


// k-DOP同士の衝突判定
// 参考ページ : 119
//
// in AMin : k-DOP Aの最小距離リスト
// in AMax : k-DOP Aの最大距離リスト
// in BMin : k-DOP Bの最小距離リスト
// in BMax : k-DOP Bの最大距離リスト
// in N    : 方向の数(配列の要素数)
// ret true-衝突あり　false:衝突なし
bool testKDOPKDOP(const float* AMin, const float* AMax, const float* BMin, const float* BMax, const size_t N) {
    for(int i=N/2; i>0; --i) {
        if(AMin[i]>BMax[i] || AMax[i] < BMin[i])
            return false;
    }

    return true;
}


// 2D線分同士の交差判定
// 参考ページ : 153
//
// in S1 : 線分1
// in S2 : 線分2
// out OutPt : 交差点出力バッファ
// out OutT  : 線分1の交差点までの割合出力バッファ
// ret true-衝突あり　false:衝突なし
bool test2DSegment2DSegment(const Segment2D& S1, const Segment2D& S2, Vector2D* OutPt, float* OutT) {
  // 最適化AABB判定
    AABB s1_aabb;
    AABB s2_aabb;
    if(!testAABBAABB(s1_aabb, s2_aabb)) return false;

  // ここからP.153
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


// 球と平面の衝突(交差)判定
// 参考ページ : 160, 161
//
// in S : 球
// in P : 平面
// ret true-交差あり　false-交差なし
//
// 純粋に、球の中心と平面の間にある距離を使って判定
// 平面と球が交差しているかどうか。完全にめりこんでいる場合は、後述の別関数で判定する
// 平面は正規化されている必要がある( |P.n| = 1 )
bool testSpherePlane(const Sphere& S, const Plane& P) {
    float dist = dot(S.c, P.n) - P.d;
    return std::abs(dist) <= S.r;
}


// 球と平面の衝突(めりこみ)判定
// 参考ページ : 161
//
// in S : 球
// in P : 平面
// ret true-めり込みあり　false-めり込みなし
//
// 球が平面の内側にあるかどうか。交差している場合はめり込みなしと判定。
// 平面は正規化されている必要がある
bool testInsideSpherePlane(const Sphere& S, const Plane& P) {
    float dist = dot(S.c, P.n) - P.d;
    return dist < -S.r;
}


// 球と平面の衝突(交差、もしくはめり込み)判定
// 参考ページ : 161
//
// in S : 球
// in P : 平面
// ret true-衝突あり　false-衝突なし
//
// 平面は正規化されている必要がある
bool testSphereHarfSpace(const Sphere& S, const Plane& P) {
    float dist = dot(S.c, P.n) - P.d;
    return dist <= S.r;
}


// OBBと平面の衝突(交差)判定
// 参考ページ : 161, 162, 163
//
// in B : OBB
// in P : 平面
// ret true-交差あり　false-交差なし
//
// 分離軸判定による実装
// 分離軸は平面の法線(P.n)に平行な線分1つのみ
// 平面は正規化されていなくともよい(最終的な比較段階で影響しない)
bool testOBBPlane(const OBB& B, const Plane& P) {
    float r = B.r[0] * std::abs(dot(P.n, B.u[0])) +
        B.r[1] + std::abs(dot(P.n, B.u[1])) +
        B.r[2] + std::abs(dot(P.n, B.u[2]));
    float s = dot(P.n, B.c) - P.d;
    return std::abs(s) <= r;
}


// OBBと平面の衝突(めり込み)判定
// 参考ページ : 162, 163
//
// in B : OBB
// in P : 平面
// ret true-めり込みあり　false-めり込みなし
bool testInsideOBBPlane(const OBB& B, const Plane& P) {
    float r = B.r[0] * std::abs(dot(P.n, B.u[0])) +
        B.r[1] + std::abs(dot(P.n, B.u[1])) +
        B.r[2] + std::abs(dot(P.n, B.u[2]));
    float s = dot(P.n, B.c) - P.d;
    return s < -r;
}


// OBBと平面の衝突(交差、めり込み)判定
// 参考ページ : 162, 163
//
// in B : OBB
// in P : 平面
// ret true-衝突あり　false-衝突なし
bool testOBBHarfSpace(const OBB& B, const Plane& P) {
    float r = B.r[0] * std::abs(dot(P.n, B.u[0])) +
        B.r[1] + std::abs(dot(P.n, B.u[1])) +
        B.r[2] + std::abs(dot(P.n, B.u[2]));
    float s = dot(P.n, B.c) - P.d;
    return s <= r;
}


// AABBと平面の衝突(交差)判定
// 参考ページ : 163
//
// in B : AABB
// in P : 平面
// ret true-交差あり　false-交差なし
bool testAABBPlane(const AABB& B, const Plane& P) {
    float r = B.r[0] * std::abs(P.n[0]) +
        B.r[1] * std::abs(P.n[1]) + 
        B.r[2] * std::abs(P.n[2]);
    float s = dot(P.n, B.c) - P.d;
    return std::abs(s) <= r;
}


// AABBと平面の衝突(めり込み)判定
//
// in B : AABB
// in P : 平面
// ret true-めり込みあり　false-めり込みなし
bool testInsideAABBPlane(const AABB& B, const Plane& P) {
    float r = B.r[0] * std::abs(P.n[0]) +
        B.r[1] * std::abs(P.n[1]) + 
        B.r[2] * std::abs(P.n[2]);
    float s = dot(P.n, B.c) - P.d;
    return s < -r;
}


// AABBと平面の衝突(交差、めり込み)判定
//
// in B : AABB
// in P : 平面
// ret true-衝突あり　false-衝突なし
bool testAABBHarfSpace(const AABB& B, const Plane& P) {
    float r = B.r[0] * std::abs(P.n[0]) +
        B.r[1] * std::abs(P.n[1]) + 
        B.r[2] * std::abs(P.n[2]);
    float s = dot(P.n, B.c) - P.d;
    return s <= r;
}


// 球とAABBの衝突判定
// 参考ページ : 165, 166
//
// in S : 球
// in B : AABB
// ret true-衝突あり　false-衝突なし
bool testSphereAABB(const Sphere& S, const AABB& B) {
    float sq_dist = sqDistPointAABB(S.c, B);
    return sq_dist <= S.r*S.r;
}


// 球とOBBの衝突判定
// 参考ページ : 166, 167
//
// in S : 球
// in B : OBB
// ret true-衝突あり　false-衝突なし
bool testSphereOBB(const Sphere& S, const OBB& B) {
    float sq_dist = sqDistPointOBB(S.c, B);
    return sq_dist <= S.r*S.r;
}


// 球と三角形の衝突判定
// 参考ページ : 167
//
// in S : 球
// in T : 三角形
// ret true-衝突あり　false-衝突なし
bool testSphereTriangle(const Sphere& S, const Triangle& T) {
    Point c = closestPtPointTriangle(S.c, T);
    return dot(c, c) <= S.r*S.r;
}


// 三角形とAABBの衝突判定
// 参考ページ : 169~172
//
// in T : 三角会
// in B : AABB
// ret true-衝突あり　false-衝突なし
//
// 分離軸判定による実装
// 1. AABBの面法線(3つ)
//      AABBと、三角形のAABBによる判定と同等
// 2. 三角形の面法線(1つ)
//      AABBと三角形の平面による交差判定と同等
// 3. AABBの軸と三角形の辺 の外積(9つ ※3*3)
//      しっかりと分離軸判定を行う
//
// 最適化 : 3->1->2 の順番で処理を行う
bool testTriangleAABB(Triangle T, const AABB& B) {
    // AABBの中心を原点とすることで計算を単純化
    T.a = T.a - B.c;
    T.b = T.b - B.c;
    T.c = T.c - B.c;

    Vector f[3] // 三角形の辺
    { T.b - T.a,
      T.c - T.b,
      T.a - T.c };


    // 3 - 分離軸判定
    float p0, p1, p2, r;
    Vector u[3] // AABBの面法線
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
    // p.p = Point{}; testAABBPlane(...)では使用しないので初期化不要
    p.n = cross(f[0], f[1]);
    p.d = dot(p.n, T.a);
    return testAABBPlane(B, p);;
}


// 平面上にある、点に最も近い点を求める
// 参考ページ : 127
//
// in Q : 点
// in P : 平面
//
// 平面の法線は正規化されている必要がある
Point closestPtPointPlane(const Point& Q, const Plane& P) {
    float t = dot(P.n, Q) - P.d;
    return Q - P.n*t;
}


// 与えられた点と平面の間にある距離を求める
// 参考ページ : 127
//
// in Q : 点
// in P : 平面
//
// 平面の法線は正規化されている必要がある
float distPointPlane(const Point& Q, const Plane& P) {
    return dot(Q, P.n) - P.d;
}


// 点と線文の最接近点を求める
// 最接近点は線分上にある
// 参考ページ : 128
//
// in P : 点
// in S : 線分
// ret 最接近点
Point closestPtPointSegment(const Point& P, const Segment& S) {
    return S.s + S.d*percentageSegmentToClosestPt(P, S);
}


// 点と線分の最接近点までの、線分の距離の割合を求める
// 割合は[0.0F : 1.0F]の範囲にクランプされる
// 参考ページ : 128
//
// in P : 点
// in S : 線分
// ret 最接近点までの、線分の距離の割合
//
// 最接近点d(t) = S.s + (S.d * t) <- tを求める
float percentageToClosestPtPointSegment(const Point& P, const Segment& S) {
    float t = dot(P-S.s, S.d) / dot(S.d, S.d);
    return clamp(t, 0.0F, 1.0F);
}


// 点と線分の間にある距離の平方を求める
// 距離は、互いの間にある最も小さい距離を指す
// 参考ページ : 130
//
// in P : 点
// in S : 線分
// ret 距離の平方
float sqDistPointSegment(const Point& P, const Segment& S) {
    Vector ac = P - S.s;
    float e = dot(ac, S.d);

    // S.s(線分の始点)が最接近点の場合
    if(e <= 0.0F) return dot(ac, ac);

    float f = dot(S.d, S.d);

    // S.s+S.d(線分の終点)が最接近点の場合
    if(e >= f) {
        Vector bc = P - (S.s+S.d);
        return dot(bc, bc);
    }

    // 最接近点がS上にある場合
    return dot(ac, ac) - e*e / f;
}


// 点とAABBの最接近点を求める
// 最接近点はAABBの上にある
// 参考ページ : 130, 131
//
// in P : 点
// in B : AABB
// ret 最接近点
//
// x,y,z 各軸について、点をAABBの範囲内にクランプする
Point closestPtPointAABB(Point P, const AABB& B) {
    for(int i=0; i<3; ++i) {
        float temp = B.c[i]-B.r[i];
        if(P[i] < temp) P[i] = temp;  else {

        temp = B.c[i]+B.r[i];
        if(P[i] > temp) P[i] = temp;  }
    }

    return P;
}


// 点とAABBの間にある距離の平方を求める
// 距離は、互いの間にある最も小さい距離を指す
// 参考ページ : 131, 132
// 
// in P : 点
// in B : AABB
// ret 距離の平方
//
// 各軸について、余分な距離の2乗を求めて加算していく
// 最終的に三平方の定理、d^2 = x^2 + y^2 + z^2 が求められる
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


// 点とOBBの最接近点を求める
// 参考ページ : 133
//
// in P : 点
// in B : OBB
// ret 最接近点
//
// 点をOBBの各軸に沿って、余分な距離だけ移動
// 点とAABBの最接近点を求めるのと考え方は同じ
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


// 点とOBBの間にある距離の平方を求める
// 距離は、互いの間にある最も小さい距離を指す
// 参考ページ : 134
//
// in P : 点
// in B : OBB
// ret 距離の平方
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


// 点と長方形の最接近点を求める
// 参考ページ : 135
//
// in P : 点
// in R : 長方形
// ret 最接近点
//
// closestPtPointOBBの応用
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


// 点と長方形の間にある距離の平方を求める
// 参考ページ : なし
//
// in P : 点
// in R : 長方形
// ret 距離の平方
float sqDistPointRectangle(const Point& P, const Rectangle& R) {
    OBB b {
        R.c,
        {R.u[0], R.u[1], cross(R.u[0], R.u[1])},
        {R.r[0], R.r[1], 0.0F}
    };
    return sqDistPointOBB(P, b);
}


// 点と三角形の最接近点を求める
// 参考ページ : 141, 142
//
// in P : 点
// in T : 三角形
// ret 最接近点
//
// 最接近点は三角形に属する
// 点がどのボロノイ領域に属するかを考える
// case 1: 頂点領域の場合、対応する頂点が最接近点
// case 2: 辺領域の場合、対応する辺に点を垂直射影した点が最接近点
// case 3: 頂点、辺のどのボロノイ領域にも属さない場合、点を三角形に垂直射影した点が最接近点
Point closestPtPointTriangle(const Point& P, const Triangle& T) {
    // PがT.aの頂点領域にあるか
    Vector ab = T.b - T.a;
    Vector ac = T.c - T.a;
    Vector ap = P - T.a;
    float d1 = dot(ab, ap);
    float d2 = dot(ac, ap);
    if(d1 <= 0.0F && d2 <= 0.0F) return T.a;    // case 1

    // PがT.bの頂点領域にあるか
    Vector bp = P - T.b;
    float d3 = dot(ab, bp);
    float d4 = dot(ac, bp);
    if(d3 >= 0.0F && d4 <= d3) return T.b;      // case 1

    // Pが線分T.a->T.bの辺領域にあるか
    float vc = d1*d4 - d3*d2;
    if(vc <= 0.0F && d1 >= 0.0F && d3 <= 0.0F)
        return T.a + ab * /*v=*/(d1/(d1-d3));   // case 2

    // PがCの頂点領域にあるか
    Vector cp = P - T.c;
    float d5 = dot(ab, cp);
    float d6 = dot(ac, cp);
    if(d6 >= 0.0F && d5 <= d6) return T.c;      // case 1

    // Pが線分T.a->T.cの辺領域にあるか
    float vb = d5*d2 - d1*d6;
    if(vb <= 0.0F && d2 >= 0.0F && d6 <= 0.0F)
        return T.a + ac * /*v=*/(d2/d2-d6);     // case 2

    // Pが線分T.b->T.cの辺領域にあるか
    float va = d3*d6 - d5*d4;
    if(va <= 0.0F && (d4-d3) >= 0.0F && (d5-d6) >= 0.0F)
        return T.b + (T.c-T.b) * /*v=*/((d4-d3)/((d4-d3) + (d5-d6))); // case 2

    float denom = 1.0F / (va+vb+vc);
    float v = vb * denom;
    float w = vc * denom;
    return T.a + ab*v + ac*w;                   // case 3
}


// 線分と線分の最接近点までの、それぞれの距離の割合を求める
//
// in S1 : 線分1
// in S2 : 線分2
// out C1 : 線分1の最接近点までの割合
// out C2 : 線分2の最接近点までの割合
void closestPtSegmentSegment(const Segment& S1, const Segment& S2, float* const C1, float* const C2) {
    Vector r = S1.s - S2.s;
    float a = dot(S1.d, S1.d);
    float e = dot(S2.d, S2.d);
    float f = dot(S2.d, r);

    if(a <= FLT_EPSILON && e <= FLT_EPSILON) {
      // 両方の線分が点に縮退
        *C1 = *C2 = 0.0F;
    }
    else if(a <= FLT_EPSILON) {
      // 線分S1が点に縮退
        *C1 = 0.0F;
        *C2 = clamp(f/e, 0.0F, 1.0F);
    }
    else if(e <= FLT_EPSILON) {
      // 線分S2が点に縮退
        *C2 = 0.0F;
        *C1 = clamp((-dot(S1.d, r))/a, 0.0F, 1.0F);
    }
    else {
      // 線分と線分の最接近点までの割合を求める
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
// 線分と線分の最接近点を求める
//
// in S1 : 線分1
// in S2 : 線分2
// out C1 : 線分1上の最接近点
// out C2 : 線分2上の最接近点
void closestPtSegmentSegment(const Segment& S1, const Segment& S2, Point* const C1, Point* const C2) {
    float s, t;
    closestPtSegmentSegment(S1, S2, &s, &t);
    *C1 = S1.s + S1.d*s;
    *C2 = S2.s + S2.d*t;
}


// 線分と線分の間にある距離の平方を求める
//
// in S1 : 線分1
// in S2 : 線分2
// ret 距離の平方
float sqDistSegmentSegment(const Segment& S1, const Segment& S2) {
    Point c1;
    Point c2;
    closestPtSegmentSegment(S1, S2, &c1, &c2);
    
    c1 = c1 - c2;
    return dot(c1, c1);
}


// 線分と三角形の最接近点を求める
// 参考ページ : 154, 155
//
// in S : 線分
// in T : 三角形
// ret 最接近点
//
// 最接近点は三角形に属する
// いくつかの考えられる組み合わせの中で、最も近い距離の点が最接近点
// 0. 線分と三角形の辺ab
// 1. 線分と三角形の辺bc
// 2. 線分と三角形の辺ca
// 3. 線分の始点と三角形の平面
// 4. 線分の終点と三角形の平面
//
// さらなる最適化の余地あり( 詳細は参考ページを参照 ) 
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

    // 最小の距離となる組み合わせを求める
    size_t min_idx=0;
    for(int i=1; i<5; ++i) {
        if(sq_dists[i] < sq_dists[min_idx])
            min_idx = i;
    }

    // 最終的に近い点のみ求める( 3,4の組み合わせは、上で求められている )
    switch(min_idx) {
    case 0 : closestPtSegmentSegment(S, Segment{T.b-T.a}, /*dummy*/&temp, &points[0]); break;
    case 1 : closestPtSegmentSegment(S, Segment{T.c-T.b}, /*dummy*/&temp, &points[1]); break;
    case 2 : closestPtSegmentSegment(S, Segment{T.a-T.c}, /*dummy*/&temp, &points[2]); break;
    };

    return points[min_idx];
}

// 三角形と三角形の最接近点を求める
// 参考ページ : 156, 157
//
// in A : 三角形1
// in B : 三角形2
// out OutP1 : 三角形1上にある最接近点の出力バッファ
// out OutP2 : 三角形2上にある最接近点の出力バッファ
// ret 最接近点ペアの間にある距離の平方
//
// いくつかの考えられる組み合わせの中で、最も近い距離の点が最接近点
// 0. 各三角形のすべての辺同士(9パターン)
// 1. 一方の頂点と他方の三角形(6パターン)
//
// さらなる頑健性の向上が求められる
// ...三角形が交差している場合、平行に重なっている場合
float closestPtTriangleTriangle(const Triangle& A, const Triangle& B, Point* OutP1, Point* OutP2) {
    struct ClosestPair{
        Point p[2];
        float sq_dist;
    };
    std::array<ClosestPair, 15U> closest_points;    // 最接近点候補のペアを格納する (15 = 9+6 ※詳しくは上記詳細を参照)

    auto segmentInTri = [](const Triangle& T, const size_t SegIdx /*1~3*/)->Segment {
        const size_t kStartIdx = SegIdx-1; // 始点
        const size_t kEndIdx   = SegIdx%3; // 終点

        // SegIdxの値に応じて 0->1, 1->2, 2->0 の組み合わせでできる線分を返す
        return Segment{T.p[kStartIdx], T.p[kEndIdx]-T.p[kStartIdx]};
    };
    // 0. 各三角形の線分同士
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

    // 1. 頂点と三角形
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


    // 最も距離が小さい点のペアを見つける
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
