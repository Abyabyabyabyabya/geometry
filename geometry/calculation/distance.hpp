#ifndef INCLUDED_GEOMETRY_CALCULATION_DISTANCE_HEADER_
#define INCLUDED_GEOMETRY_CALCULATION_DISTANCE_HEADER_

#include <cmath>
#include <utility>
#include <type_traits>
#include "../utility/access.hpp"
#include "../traits/traits.hpp"
#include "../traits/point_traits.hpp"

namespace geometry {

template <class T=float>
struct DistanceResult {
    T sq_dist;
    operator T() const noexcept(noexcept(std::sqrt(sq_dist))) {
        return std::sqrt(sq_dist);
    }
};
template <class D1, class D2>
inline bool operator< (const DistanceResult<D1> R1, const DistanceResult<D2> R2) noexcept { return R1.sq_dist < R2.sq_dist; }
template <class D1, class D2>
inline bool operator> (const DistanceResult<D1> R1, const DistanceResult<D2> R2) noexcept { return R1.sq_dist > R2.sq_dist; }

  namespace distance_impl {
    template <class G1, class G2, class G1Tag=traits::Category_t<G1>, class G2Tag=traits::Category_t<G2>>
    struct Distance;
  } // namespace distance_impl

template <class G1, class G2>
inline auto distance(const G1& G1_, const G2& G2_) noexcept(noexcept(distance_impl::Distance<G1, G2>::apply(G1_, G2_))) {
    return distance_impl::Distance<G1, G2>::apply(G1_, G2_);
}


namespace distance_impl {

template <class P1, class P2, size_t Dim>
//
struct Pythagoras {
    using return_type = decltype(std::declval<traits::ValueType_t<P1>>()*std::declval<traits::ValueType_t<P2>>());
    static return_type apply(const P1& P1_, const P2& P2_) noexcept(
      noexcept(std::is_nothrow_constructible_v<return_type>) &&
      noexcept(utility::get<Dim-1>(P1_)-utility::get<Dim-1>(P2_))&&
      noexcept(utility::get<Dim-1>(P1_)*utility::get<Dim-1>(P2_))) {
        using namespace traits;
        return_type d = utility::get<Dim-1>(P1_) - utility::get<Dim-1>(P2_);
        return (d*d) + Pythagoras<P1, P2, Dim-1>::apply(P1_, P2_);
    }
};
template <class P1, class P2>
struct Pythagoras<P1, P2, 0> {
    using return_type = typename Pythagoras<P1, P2, 1>::return_type;
    static return_type apply(const P1& P1_, const P2& P2_) noexcept(std::is_nothrow_constructible_v<return_type>) {
        return return_type{};
    }
};

// distance of (point, point)
template <class P1, class P2>
struct Distance<P1, P2, traits::PointCategory, traits::PointCategory> {
    static auto apply(const P1& P1_, const P2& P2_) noexcept(noexcept(Pythagoras<P1, P2, traits::Dimension_v<P1>>::apply(P1_, P2_))) {
        using namespace traits;
        static_assert(Dimension_v<P1> == Dimension_v<P2>);
        using Pythagoras_ = Pythagoras<P1, P2, Dimension_v<P1>>;
        return DistanceResult<typename Pythagoras_::return_type>{Pythagoras_::apply(P1_, P2_)};
    }
};

// distance of (point, segment)
template <class P, class S>
struct Distance<P, S, traits::PointCategory, traits::SegmentCategory> {

};
} // namespace distance_impl

} // namespace geometry

#endif // !INCLUDED_GEOMETRY_CALCULATION_DISTANCE_HEADER_
// EOF
