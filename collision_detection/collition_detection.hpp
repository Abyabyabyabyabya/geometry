
#ifndef INCLUDED_COLLISION_DETECTION_HEADER_
#define INCLUDED_COLLISION_DETECTION_HEADER_


#include <cstdlib>
#include "../common/aabb.hpp"
#include "../common/sphere.hpp"
#include "../common/obb.hpp"
#include "../common/capsule.hpp"
#include "../common/segment.hpp"
#include "../common/kdop.hpp"
#include "../common/plane.hpp"
#include "../common/rectangle.hpp"
#include "../common/triangle.hpp"


float signed2DTriArea(const Triangle2D&);

bool testAABBAABB(const AABB&, const AABB&);
bool testSphereSphere(const Sphere&, const Sphere&);
bool testOBBOBB(const OBB&, const OBB&);
bool testSphereCapsule(const Sphere&, const Capsule&);
bool testCapsuleCapsule(const Capsule&, const Capsule&);
bool testKDOPKDOP(const float*, const float*, const float*, const float*, size_t);
template <unsigned K>
bool testKDOPKDOP(const KDOP<K>& A, const KDOP<K>& B) { return testKDOPKDOP(A.min, A.max, B.min, B.max, K); }
bool test2DSegment2DSegment(const Segment2D&, const Segment2D&, Vector2D* =nullptr, float* =nullptr);
bool testSpherePlane(const Sphere&, const Plane&);
bool testInsideSpherePlane(const Sphere&, const Plane&);
bool testSphereHarfSpace(const Sphere&, const Plane&);
bool testOBBPlane(const OBB&, const Plane&);
bool testInsideOBBPlane(const OBB&, const Plane&);
bool testOBBHarfSpace(const OBB&, const Plane&);
bool testAABBPlane(const AABB&, const Plane&);
bool testInsideAABBPlane(const AABB&, const Plane&);
bool testAABBHarfSpace(const AABB&, const Plane&);
bool testSphereAABB(const Sphere&, const AABB&);
bool testSphereOBB(const Sphere&, const OBB&);
bool testSphereTriangle(const Sphere&, const Triangle&);
bool testTriangleAABB(Triangle, const AABB&);

Point closestPtPointPlane(const Point&, const Plane&);
float distPointPlane(const Point&, const Plane&);
Point closestPtPointSegment(const Point&, const Segment&);
float percentageSegmentToClosestPt(const Point&, const Segment&);
float sqDistPointSegment(const Point&, const Segment&);
Point closestPtPointAABB(Point, const AABB&);
float sqDistPointAABB(const Point&, const AABB&);
Point closestPtPointOBB(Point, const OBB&);
float sqDistPointOBB(const Point&, const OBB&);
Point closestPtPointRect(Point, const Rectangle&);
float sqDistPointRect(const Point&, const Rectangle&);
Point closestPtPointTriangle(const Point&, const Triangle&);
void closestPtSegmentSegment(const Segment&, const Segment&, float*, float*);
void closestPtSegmentSegment(const Segment&, const Segment&, Point*, Point*);
float sqDistSegmentSegment(const Segment&, const Segment&);
Point closestPtSegmentTriangle(const Segment&, const Triangle&);
float closestPtTriangleTriangle(const Triangle&, const Triangle&, Point*, Point*);

#endif // !INCLUDED_COLLISION_DETECTION_HEADER_
// EOF
