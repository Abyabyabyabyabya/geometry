
#ifndef INCLUDED_KDOP_HEADER_
#define INCLUDED_KDOP_HEADER_

#include <type_traits>

template <unsigned K>
struct KDOP {
    static_assert(!(K%2), "K‚Í‹ô”‚Å‚È‚­‚Ä‚Í‚È‚ç‚È‚¢");

    float min[K];
    float max[K];
};

#endif // !INCLUDED_KDOP_HEADER_
// EOF
