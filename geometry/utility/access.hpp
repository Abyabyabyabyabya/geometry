#ifndef INCLUDED_GEOMETRY_UTILITY_ACCESS_HEADER
#define INCLUDED_GEOMETRY_UTILITY_ACCESS_HEADER

#include "../traits/traits.hpp"

#define DEF_ACCESS(Traits, Dim, Func) \
template <class G> struct geometry::utility::Access<Traits<G>, Dim> { \
static auto get(const G& G_) noexcept(noexcept(Traits<G>::Func(G_))) { \
return Traits<G>::Func(G_); } }

namespace geometry {
namespace utility {

template <class Traits, size_t D>
struct Access;

template <size_t D, class G>
inline auto get(const G& G_) noexcept(noexcept(Access<traits::Traits_t<G>, D>::get(G_))) {
    return Access<traits::Traits_t<G>, D>::get(G_);
}

} // namespace utility
} // namespace geometry

#endif // !INCLUDED_GEOMETRY_UTILITY_ACCESS_HEADER
// EOF
