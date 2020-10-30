
#include "calc.hpp"

Point operator+(const Point& A, const Point& B) {
    return Point{A.x+B.x, A.y+B.y, A.z+B.z};
}

Point operator-(const Point& A, const Point& B) {
    return Point{A.x-B.x, A.y-B.y, A.z-B.z};
}

Point operator*(const Point& A, const float B) {
    return Point{A.x*B, A.y*B, A.z*B};
}


float dot(const Point& A, const Point& B) {
    return (A.x*B.x) + (A.y*B.y) + (A.z*B.z);
}

Point cross(const Point& A, const Point& B) {
    return Point {
        A.y*B.z - B.y*A.z,
        A.z*B.x - B.z*A.x,
        A.x*B.y - B.x*A.y
    };
}

Vector2D operator+(const Vector2D& A, const Vector2D& B) {
    return Vector2D{A.x+B.x, A.y+B.y};
}

Vector2D operator-(const Vector2D& A, const Vector2D& B) {
    return Vector2D{A.x-B.x, A.y-B.y};
}

Vector2D operator*(const Vector2D& A, const float B) {
    return Vector2D{A.x*B, A.y*B};
}

// EOF
