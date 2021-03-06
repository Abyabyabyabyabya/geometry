
#ifndef INCLUDED_KDOP_HEADER_
#define INCLUDED_KDOP_HEADER_

#include <type_traits>

template <unsigned K>
struct KDOP {
    static_assert(!(K%2), "Kは偶数でなくてはならない");

    float min[K];
    float max[K];
};

#endif // !INCLUDED_KDOP_HEADER_
// EOF
