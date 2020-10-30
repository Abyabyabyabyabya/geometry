#ifndef INCLUDED_GEOMETRY_GEOMETRIES_POINT_TRAITS_HEADER_
#define INCLUDED_GEOMETRY_GEOMETRIES_POINT_TRAITS_HEADER_

#include <type_traits>
#include <utility>
#include "traits.hpp"
#include "../utility/access.hpp"
#include "../utility/utility.hpp"

namespace geometry {
namespace traits {

template <class P>
struct PointTraits {
    static auto x(const P& P_) noexcept(noexcept(P_.x())) {
        return P_.x();
    }
    static auto y(const P& P_) noexcept(noexcept(P_.y())) -> decltype(x(std::declval<P>())) {
        return P_.y();
    }
    template <decltype(std::declval<P>().z(), int{})* =nullptr>
    static auto z(const P& P_) noexcept(noexcept(P_.z())) -> decltype(x(std::declval<P>())) {
        return P_.z();
    }
    template <decltype(std::declval<P>().w(), int{})* =nullptr>
    static auto w(const P& P_) noexcept(noexcept(P_.w())) -> decltype(x(std::declval<P>())) {
        return P_.w();
    }
};

  namespace impl {
    template <class P>
    class PointDimensionImpl {
        struct Z {
            template <decltype(geometry::traits::PointTraits<P>::z(std::declval<P>()), int{})* =nullptr>
            void operator()(const P&) noexcept {}
        };
        struct W {
            template <decltype(geometry::traits::PointTraits<P>::w(std::declval<P>()), int{})* =nullptr>
            void operator()(const P&) noexcept {}
        };
        static constexpr bool invocable_z = std::is_invocable_v<Z, const P&>;
        static constexpr bool invocable_w = std::is_invocable_v<W, const P&>;
    public :
        static constexpr size_t value = !invocable_z ? 2 : !invocable_w ? 3 : 4;
    };
  } // namespace impl

template <class P>
struct PointDimension : PointDimensionImpl<PointTraits<P>> {};
template <class P>
static constexpr size_t PointDimension_v = PointDimensionImpl<PointTraits<P>>::value;

} // namespace traits
} // namespace geometry


// template specialization
template <> 
struct geometry::traits::impl::TraitsImpl<geometry::traits::PointCategory> :
    utility::MetaTemplateType<PointTraits> {};
template <class P>
struct geometry::traits::impl::DimensionImpl<geometry::traits::PointTraits<P>> :
    PointDimension<PointTraits<P>> {};
template <class P>
struct geometry::traits::impl::ValueTypeImpl<geometry::traits::PointTraits<P>> :
    utility::MetaType<decltype(PointTraits<P>::x(std::declval<P>()))> {};
template <class P>
struct geometry::traits::impl::PointTypeImpl<geometry::traits::PointTraits<P>> :
    utility::MetaType<P> {};

DEF_ACCESS(geometry::traits::PointTraits, 0, x);
DEF_ACCESS(geometry::traits::PointTraits, 1, y);
DEF_ACCESS(geometry::traits::PointTraits, 2, z);
DEF_ACCESS(geometry::traits::PointTraits, 3, w);

#endif // !INCLUDED_GEOMETRY_GEOMETRIES_POINT_TRAITS_HEADER_
// EOF
