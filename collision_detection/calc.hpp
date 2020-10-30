
#ifndef INCLUDED_CALC_HEADER_
#define INCLUDED_CALC_HEADER_

#include "../common/vector.hpp"

Point operator+(const Point&, const Point&);
Point operator-(const Point&, const Point&);
Point operator*(const Point&, float);
float dot(const Point&, const Point&);
Point cross(const Point&, const Point&);

Vector2D operator+(const Vector2D&, const Vector2D&);
Vector2D operator-(const Vector2D&, const Vector2D&);
Vector2D operator*(const Vector2D&, float);

#endif // !INCLUDED_CALC_HEADER_
// EOF
