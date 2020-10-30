
#ifndef INCLUDED_TRIANGLE_HEADER_
#define INCLUDED_TRIANGLE_HEADER_

#include "vector.hpp"

struct Triangle {
    union {
        Point p[3];
        struct {
            Point a;
            Point b;
            Point c;
        };
    };
};

struct Triangle2D {
    union {
        Vector2D d[3];
        struct {
            Vector2D a;
            Vector2D b;
            Vector2D c;
        };
    };
};

#endif // !INCLUDED_TRIANGLE_HEADER_
// EOF
